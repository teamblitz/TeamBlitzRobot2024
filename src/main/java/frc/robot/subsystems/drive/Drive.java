/* Big thanks to Team 364 for the base swerve code. */

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.Drive.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.Subscriber;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.BlitzSubsystem;
import frc.lib.util.LimelightHelpers;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.drive.swerveModule.SwerveModule;
import frc.robot.subsystems.drive.swerveModule.SwerveModuleConfiguration;
import frc.robot.subsystems.drive.swerveModule.angle.AngleMotorIOSpark;
import frc.robot.subsystems.drive.swerveModule.drive.DriveMotorIOKraken;
import frc.robot.subsystems.drive.swerveModule.drive.DriveMotorIOSpark;
import frc.robot.subsystems.drive.swerveModule.encoder.EncoderIOCanCoder;
import frc.robot.subsystems.drive.swerveModule.encoder.EncoderIOHelium;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Here we can probably do some cleanup, main thing we can probably do here is separate
 * telemetry/hardware io. Also, we need a better way to do dynamic pid loop tuning.
 */
public class Drive extends SubsystemBase implements BlitzSubsystem {
    private final SwerveDriveOdometry swerveOdometry;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveModule[] swerveModules;
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drive");
    private final ShuffleboardTab tuningTab = Shuffleboard.getTab("DriveTuning");
    private final ShuffleboardLayout anglePidLayout =
            tuningTab.getLayout("AnglePid", BuiltInLayouts.kList);
    private final ShuffleboardLayout drivePidLayout =
            tuningTab.getLayout("DrivePid", BuiltInLayouts.kList);
    private final Field2d field = new Field2d();

    private final GenericEntry anglePEntry =
            anglePidLayout.add("angleP", ANGLE_KP).getEntry("double");
    private final GenericEntry angleIEntry =
            anglePidLayout.add("angleI", ANGLE_KI).getEntry("double");
    private final GenericEntry angleDEntry =
            anglePidLayout.add("angleD", ANGLE_KD).getEntry("double");

    private final GenericEntry drivePEntry =
            drivePidLayout.add("driveP", DRIVE_KP).getEntry("double");
    private final GenericEntry driveIEntry =
            drivePidLayout.add("driveI", DRIVE_KI).getEntry("double");
    private final GenericEntry driveDEntry =
            drivePidLayout.add("driveD", DRIVE_KD).getEntry("double");

    private double angleP = ANGLE_KP;
    private double angleI = ANGLE_KI;
    private double angleD = ANGLE_KD;

    private double driveP = DRIVE_KP;
    private double driveI = DRIVE_KI;
    private double driveD = DRIVE_KD;

    private double lastTurnCommandSeconds;
    private boolean keepHeadingSetpointSet;

    private final PIDController keepHeadingPid;
    private final ProfiledPIDController rotateToHeadingPid;

    private SysIdRoutine routine;

    private double lastVisionTimeStamp;

    private Rotation2d gyroOffset = new Rotation2d();
    private final NetworkTableEntry limelightPose = LimelightHelpers.getLimelightNTTableEntry("limelight", "botpose_wpiblue");

    public Drive(
            SwerveModuleConfiguration configuration,
            SwerveModuleConstants flConstants,
            SwerveModuleConstants frConstants,
            SwerveModuleConstants blConstants,
            SwerveModuleConstants brConstants,
            GyroIO gyroIO) {
        this(
                new SwerveModule(
                        FL,
                        new AngleMotorIOSpark(flConstants),
                        configuration.drive == SwerveModuleConfiguration.MotorType.KRAKEN
                                ? new DriveMotorIOKraken(flConstants)
                                : new DriveMotorIOSpark(flConstants),
                        configuration.encoder == SwerveModuleConfiguration.EncoderType.CANCODER
                                ? new EncoderIOCanCoder(flConstants.cancoderID, CAN_CODER_INVERT)
                                : new EncoderIOHelium(flConstants.cancoderID, CAN_CODER_INVERT)),
                new SwerveModule(
                        FR,
                        new AngleMotorIOSpark(frConstants),
                        configuration.drive == SwerveModuleConfiguration.MotorType.KRAKEN
                                ? new DriveMotorIOKraken(frConstants)
                                : new DriveMotorIOSpark(frConstants),
                        configuration.encoder == SwerveModuleConfiguration.EncoderType.CANCODER
                                ? new EncoderIOCanCoder(frConstants.cancoderID, CAN_CODER_INVERT)
                                : new EncoderIOHelium(frConstants.cancoderID, CAN_CODER_INVERT)),
                new SwerveModule(
                        BL,
                        new AngleMotorIOSpark(blConstants),
                        configuration.drive == SwerveModuleConfiguration.MotorType.KRAKEN
                                ? new DriveMotorIOKraken(blConstants)
                                : new DriveMotorIOSpark(blConstants),
                        configuration.encoder == SwerveModuleConfiguration.EncoderType.CANCODER
                                ? new EncoderIOCanCoder(blConstants.cancoderID, CAN_CODER_INVERT)
                                : new EncoderIOHelium(blConstants.cancoderID, CAN_CODER_INVERT)),
                new SwerveModule(
                        BR,
                        new AngleMotorIOSpark(brConstants),
                        configuration.drive == SwerveModuleConfiguration.MotorType.KRAKEN
                                ? new DriveMotorIOKraken(brConstants)
                                : new DriveMotorIOSpark(brConstants),
                        configuration.encoder == SwerveModuleConfiguration.EncoderType.CANCODER
                                ? new EncoderIOCanCoder(brConstants.cancoderID, CAN_CODER_INVERT)
                                : new EncoderIOHelium(brConstants.cancoderID, CAN_CODER_INVERT)),
                gyroIO);
    }

    public Drive(
            SwerveModule frontLeft,
            SwerveModule frontRight,
            SwerveModule backLeft,
            SwerveModule backRight,
            GyroIO gyroIO) {
        swerveModules =
                new SwerveModule[] { // front left, front right, back left, back right.
                    frontLeft, frontRight, backLeft, backRight
                };

        swerveOdometry = new SwerveDriveOdometry(KINEMATICS, getYaw(), getModulePositions());
        poseEstimator =
                new SwerveDrivePoseEstimator(
                        KINEMATICS, getYaw(), getModulePositions(), new Pose2d());

        this.gyroIO = gyroIO;

        keepHeadingPid = new PIDController(.15, 0, 0);
        keepHeadingPid.enableContinuousInput(-180, 180);
        keepHeadingPid.setTolerance(2);

        rotateToHeadingPid = new ProfiledPIDController(.1, 0, 0, new Constraints(180, 360));
        rotateToHeadingPid.enableContinuousInput(-180, 180);
        keepHeadingPid.setTolerance(2);
        initTelemetry();

        zeroGyro();

        new Trigger(DriverStation::isEnabled)
                .onTrue(Commands.runOnce(() -> keepHeadingSetpointSet = false));

        // Most critical 6 lines of the robot, don't delete, without these it doesn't completely work
        // for some reason
        Commands.waitSeconds(3)
                .andThen(
                        Commands.runOnce(
                                        () -> {
                                            for (SwerveModule module : swerveModules) {
                                                module.resetToAbs();
                                            }
                                        })
                                .ignoringDisable(true))
                .schedule();

        // Creates a SysIdRoutine
        routine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                null,
                                Seconds.of(5),
                                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
                        new SysIdRoutine.Mechanism(
                                (volts) -> {
                                    drive(
                                            new Translation2d(volts.in(Volts) / 12.0, 0)
                                                    .times(Constants.Drive.MAX_SPEED),
                                            0,
                                            false,
                                            true,
                                            true);
                                },
                                log -> {
                                    log.motor("drive")
                                            .voltage(Volts.of(swerveModules[0].getVoltsDrive()))
                                            .linearVelocity(MetersPerSecond.of(swerveModules[0].getVelocity()))
                                            .linearPosition(Meters.of(swerveModules[0].getPositionDrive()));
                                },
                                this));

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetOdometry,
                () -> KINEMATICS.toChassisSpeeds(getModuleStates()),
                (speeds) ->
                        drive(speeds, false),
                Constants.AutoConstants.HOLONOMIC_PATH_FOLLOWER_CONFIG,
                () ->
                        DriverStation.getAlliance().isPresent()
                                && DriverStation.getAlliance()
                                        .get()
                                        .equals(DriverStation.Alliance.Red),
                this);
    }

    public void drive(
            Translation2d translation,
            double rotation,
            boolean fieldRelative,
            boolean isOpenLoop,
            boolean maintainHeading) {

        angleDrive(translation, rotation, 0, fieldRelative, isOpenLoop, maintainHeading, false);
    }

    public void angleDrive(
            Translation2d translation,
            double rotation,
            double rotationSetpoint,
            boolean fieldRelative,
            boolean isOpenLoop,
            boolean maintainHeading,
            boolean doRotationPid) {
        if (doRotationPid) {
            keepHeadingSetpointSet = false;

            rotation = rotateToHeadingPid.calculate(getYaw().getDegrees(), rotationSetpoint);
        } else {
            if (rotation != 0) {
                lastTurnCommandSeconds = Timer.getFPGATimestamp();
                keepHeadingSetpointSet = false;
                Logger.recordOutput("Drive/Turning", true);
            }
            if (lastTurnCommandSeconds + .5 <= Timer.getFPGATimestamp()
                    && !keepHeadingSetpointSet) { // If it has been at least .5 seconds.
                keepHeadingPid.setSetpoint(getYaw().getDegrees());
                keepHeadingSetpointSet = true;
                Logger.recordOutput("Drive/Turning", false);
            }

            if (keepHeadingSetpointSet && maintainHeading) {
                rotation = keepHeadingPid.calculate(getYaw().getDegrees());
            }
        }

        Logger.recordOutput("Drive/keepHeadingSetpointSet", keepHeadingSetpointSet);
        Logger.recordOutput("Drive/keepSetpoint", keepHeadingPid.getSetpoint());
        
        
        drive(fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getYaw())
                : new ChassisSpeeds(
                translation.getX(), translation.getY(), rotation),
                isOpenLoop
        );
    }
    
    public void drive(ChassisSpeeds speeds, boolean openLoop) {
        SwerveModuleState[] swerveModuleStates =
                KINEMATICS.toSwerveModuleStates(
                        speeds);
        
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, MAX_SPEED
        );
        
        setModuleStates(swerveModuleStates, openLoop, false, false);
    }

    /* Used by SwerveControllerCommand in Auto */
    // Use in above method?
    public void setModuleStates(
            SwerveModuleState[] desiredStates, boolean openLoop, boolean tuning, boolean parking) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_SPEED);

        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], openLoop, tuning, parking);
        }
    }

    public void park() {
        SwerveModuleState[] desiredStates = {
            (new SwerveModuleState(0, Rotation2d.fromDegrees(45))),
            (new SwerveModuleState(0, Rotation2d.fromDegrees(-45))),
            (new SwerveModuleState(0, Rotation2d.fromDegrees(-45))),
            (new SwerveModuleState(0, Rotation2d.fromDegrees(45)))
        };

        setModuleStates(desiredStates, true, false, true);
    }

    public void setBrakeMode(boolean enabled) {
        for (SwerveModule swerveModule : swerveModules) {
            swerveModule.setBrakeMode(enabled);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : swerveModules) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    public ChassisSpeeds getFieldRelativeSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getYaw());
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            swerveModules[0].getPosition(),
            swerveModules[1].getPosition(),
            swerveModules[2].getPosition(),
            swerveModules[3].getPosition()
        };
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Pose2d getLimelightPose() {
        return LimelightHelpers.getBotPose2d_wpiBlue("limelight");
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
        poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public void resetPose(Pose2d pose) {
        swerveOdometry.resetPosition(new Rotation2d(), getModulePositions(), pose);
    }

    public void zeroGyro() {
        setGyro(0);
    }

    public void setGyro(double degrees) {
        gyroOffset = Rotation2d.fromDegrees(degrees).minus(Rotation2d.fromDegrees(gyroInputs.yaw));
        keepHeadingSetpointSet = false;
        lastTurnCommandSeconds = Timer.getFPGATimestamp();
    }

    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(gyroInputs.yaw).plus(gyroOffset);
    }

    public double getPitch() {
        return gyroInputs.pitch;
    }

    public double getRoll() {
        return gyroInputs.roll;
    }

    @Override
    public void periodic() {
        for (SwerveModule mod : swerveModules) {
            mod.periodic();
        }
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("gyro", gyroInputs);

        swerveOdometry.update(getYaw(), getModulePositions());
        poseEstimator.update(getYaw(), getModulePositions());


        /* Vision stuff no touchy*/

        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

        if(
                (limelightMeasurement.tagCount >= 2
                        || (limelightMeasurement.avgTagDist < 3 && limelightMeasurement.tagCount >=1)
                        ) // maybe this will work for the amp, I am unconvinced
                        && limelightMeasurement.timestampSeconds > lastVisionTimeStamp)  {
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999)); // Standard deviations, basically vision measurements very up to .7m, and just don't trust the vision angle at all
            poseEstimator.addVisionMeasurement(
                    limelightMeasurement.pose,
                    limelightMeasurement.timestampSeconds);
        }

        lastVisionTimeStamp = limelightMeasurement.timestampSeconds;



        Logger.recordOutput("Drive/Odometry", swerveOdometry.getPoseMeters());
        Logger.recordOutput("Drive/Vision+Odometry", poseEstimator.getEstimatedPosition());
        Logger.recordOutput("Drive/Vision", getLimelightPose());
        Logger.recordOutput("Drive/modules", getModuleStates());

        boolean anglePIDChanged = false;
        boolean drivePIDChanged = false;

        if (anglePEntry.getDouble(angleP) != angleP) {
            anglePIDChanged = true;
            angleP = anglePEntry.getDouble(angleP);
        }
        if (angleIEntry.getDouble(angleI) != angleI) {
            anglePIDChanged = true;
            angleI = angleIEntry.getDouble(angleI);
        }
        if (angleDEntry.getDouble(angleD) != angleD) {
            anglePIDChanged = true;
            angleD = angleDEntry.getDouble(angleD);
        }

        if (drivePEntry.getDouble(driveP) != driveP) {
            drivePIDChanged = true;
            driveP = drivePEntry.getDouble(driveP);
        }
        if (driveIEntry.getDouble(driveI) != driveI) {
            drivePIDChanged = true;
            driveI = driveIEntry.getDouble(driveI);
        }
        if (driveDEntry.getDouble(driveD) != driveD) {
            drivePIDChanged = true;
            driveD = driveDEntry.getDouble(driveD);
        }
//        anglePIDChanged = false;
//        drivePIDChanged = false;

        if (drivePIDChanged) {
            for (SwerveModule module : swerveModules) {
                module.configDrivePid(driveP, driveI, driveD);
            }
        }
        if (anglePIDChanged) {
            for (SwerveModule module : swerveModules) {
                module.configAnglePid(angleP, angleI, angleD);
            }
        }
    }

    @Override
    public void simulationPeriodic() {
        drawRobotOnField(field);
    }

    public void initTelemetry() {
        shuffleboardTab.add(field);
        tuningTab.add("KeepHeadingPid", keepHeadingPid);
        // tuningTab.add("Tuning Command", new SwerveTuning(this));
    }

    public void drawRobotOnField(Field2d field) {
        field.setRobotPose(getPose());
        // Draw a pose that is based on the robot pose, but shifted by the translation of the module
        // relative to robot center,
        // then rotated around its own center by the angle of the module.
        SwerveModuleState[] swerveModuleStates = getModuleStates();
        field.getObject("frontLeft")
                .setPose(
                        getPose()
                                .transformBy(
                                        new Transform2d(
                                                CENTER_TO_MODULE.get(FL),
                                                swerveModuleStates[FL].angle)));
        field.getObject("frontRight")
                .setPose(
                        getPose()
                                .transformBy(
                                        new Transform2d(
                                                CENTER_TO_MODULE.get(FR),
                                                swerveModuleStates[FR].angle)));
        field.getObject("backLeft")
                .setPose(
                        getPose()
                                .transformBy(
                                        new Transform2d(
                                                CENTER_TO_MODULE.get(BL),
                                                swerveModuleStates[BL].angle)));
        field.getObject("backRight")
                .setPose(
                        getPose()
                                .transformBy(
                                        new Transform2d(
                                                CENTER_TO_MODULE.get(BR),
                                                swerveModuleStates[BR].angle)));
    }

    public Command buildParkCommand() {
        return Commands.runOnce(this::park, this);
    }

    public Command driveSpeedTestCommand(double speed, double duration) {
        SlewRateLimiter filter = new SlewRateLimiter(1);
        return Commands.run(
                        () ->
                                drive(
                                        new Translation2d(filter.calculate(speed), 0),
                                        0,
                                        false,
                                        false,
                                        false))
                .withTimeout(duration)
                .andThen(
                        Commands.run(
                                () ->
                                        drive(
                                                new Translation2d(filter.calculate(0), 0),
                                                0,
                                                false,
                                                false,
                                                false)));
    }

    /**
     * Chase a field relative vector
     * @param vector robot relative unit vector to move in the direction of
     * @param angle angle error
     * @param velocity goal velocity m/s
     * @param acceleration max acceleration m/s^2
     * @return Chase Vector Command.
     */
    public Command chaseVector(Supplier<Translation2d> vector, DoubleSupplier angle, double velocity, double acceleration) {
        SlewRateLimiter xLimiter = new SlewRateLimiter(acceleration);
        SlewRateLimiter yLimiter = new SlewRateLimiter(acceleration);

        return Commands.runOnce(
                () -> {
                    xLimiter.reset(getFieldRelativeSpeeds().vxMetersPerSecond);
                    yLimiter.reset(getFieldRelativeSpeeds().vyMetersPerSecond);
                }
        ).andThen(
                run(
                        () -> {
                            Translation2d unitVec = vector.get().div(vector.get().getNorm());
                            Translation2d goalSpeeds = unitVec.times(velocity);


                            angleDrive(
                                    new Translation2d(
                                            xLimiter.calculate(goalSpeeds.getX()),
                                            yLimiter.calculate(goalSpeeds.getY())
                                    ),
                                    angle.getAsDouble() * .0 * MAX_ANGULAR_VELOCITY,
                                    0,
                                    true,
                                    true,
                                    true,
                                    false
                            );
                        }
                )
        );
    }

    public Command zeroAbsEncoders() {
        return runOnce(
                        () -> {
                            swerveModules[0].zeroAbsEncoders();
                            swerveModules[1].zeroAbsEncoders();
                            swerveModules[2].zeroAbsEncoders();
                            swerveModules[3].zeroAbsEncoders();
                        })
                .ignoringDisable(true);
    }

    // Something something super class???
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }
}
