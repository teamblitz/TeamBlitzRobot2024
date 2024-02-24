/* Big thanks to Team 364 for the base swerve code. */

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.Swerve.ANGLE_KD;
import static frc.robot.Constants.Swerve.ANGLE_KI;
import static frc.robot.Constants.Swerve.ANGLE_KP;
import static frc.robot.Constants.Swerve.BL;
import static frc.robot.Constants.Swerve.BR;
import static frc.robot.Constants.Swerve.CAN_CODER_INVERT;
import static frc.robot.Constants.Swerve.CENTER_TO_MODULE;
import static frc.robot.Constants.Swerve.DRIVE_KD;
import static frc.robot.Constants.Swerve.DRIVE_KI;
import static frc.robot.Constants.Swerve.DRIVE_KP;
import static frc.robot.Constants.Swerve.FL;
import static frc.robot.Constants.Swerve.FR;
import static frc.robot.Constants.Swerve.KINEMATICS;
import static frc.robot.Constants.Swerve.MAX_SPEED;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
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
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.BlitzSubsystem;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.drive.swerveModule.SwerveModule;
import frc.robot.subsystems.drive.swerveModule.SwerveModuleConfiguration;
import frc.robot.subsystems.drive.swerveModule.angle.AngleMotorIOSpark;
import frc.robot.subsystems.drive.swerveModule.drive.DriveMotorIOSpark;
import frc.robot.subsystems.drive.swerveModule.encoder.EncoderIOHelium;

/**
 * Here we can probably do some cleanup, main thing we can probably do here is separate
 * telemetry/hardware io. Also, we need a better way to do dynamic pid loop tuning.
 */
public class Drive extends SubsystemBase implements BlitzSubsystem {
    private final SwerveDriveOdometry swerveOdometry;
    private final SwerveModule[] swerveModules;
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Logger logger;
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
                        new DriveMotorIOSpark(flConstants),
                        new EncoderIOHelium(flConstants.cancoderID, CAN_CODER_INVERT)),
                new SwerveModule(
                        FR,
                        new AngleMotorIOSpark(frConstants),
                        new DriveMotorIOSpark(frConstants),
                        new EncoderIOHelium(frConstants.cancoderID, CAN_CODER_INVERT)),
                new SwerveModule(
                        BL,
                        new AngleMotorIOSpark(blConstants),
                        new DriveMotorIOSpark(blConstants),
                        new EncoderIOHelium(blConstants.cancoderID, CAN_CODER_INVERT)),
                new SwerveModule(
                        BR,
                        new AngleMotorIOSpark(brConstants),
                        new DriveMotorIOSpark(brConstants),
                        new EncoderIOHelium(brConstants.cancoderID, CAN_CODER_INVERT)),
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
        this.gyroIO = gyroIO;
        logger = Logger.getInstance();

        keepHeadingPid = new PIDController(.1, 0, 0);
        keepHeadingPid.enableContinuousInput(-180, 180);

        rotateToHeadingPid = new ProfiledPIDController(.1, 0, 0, new Constraints(360, 360 * 3));
        rotateToHeadingPid.enableContinuousInput(-180, 180);
        initTelemetry();

        // In theory ignoring disabled is unnecessary, but this is a critical command that must run.
        new Trigger(DriverStation::isAutonomousEnabled)
                .onTrue(Commands.runOnce(() -> gyroIO.preMatchZero(180)).ignoringDisable(true));

        gyroIO.preMatchZero(0);

        new Trigger(DriverStation::isEnabled)
                .onTrue(Commands.runOnce(() -> keepHeadingSetpointSet = false));

        // Most critical 6 lines of the robot, don't delete, without these it doesn't completly work for some reason
        Commands.waitSeconds(3)
                .andThen(Commands.runOnce(() -> {
                    for (SwerveModule module : swerveModules) {
                        module.resetToAbs();
                    }
                }).ignoringDisable(true)).schedule();


        // Creates a SysIdRoutine
        routine = new SysIdRoutine(
                new SysIdRoutine.Config(null, null, Seconds.of(5), (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
                new SysIdRoutine.Mechanism((volts) -> {
                    drive(
                            new Translation2d(volts.in(Units.Volts)/12.0, 0).times(Constants.Swerve.MAX_SPEED),
                            0,
                            false,
                            true,
                            true
                    );
                }, null, this)
        );

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetOdometry,
                () -> KINEMATICS.toChassisSpeeds(getModuleStates()),
                (states) ->
                        setModuleStates(KINEMATICS.toSwerveModuleStates(states), false, false, false),
                Constants.AutoConstants.HOLONOMIC_PATH_FOLLOWER_CONFIG,
                () -> true,
                this
        );
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
                logger.recordOutput("Swerve/Turning", true);
            }
            if (lastTurnCommandSeconds + .5 <= Timer.getFPGATimestamp()
                    && !keepHeadingSetpointSet) { // If it has been at least .5 seconds.
                keepHeadingPid.setSetpoint(getYaw().getDegrees());
                keepHeadingSetpointSet = true;
                logger.recordOutput("Swerve/Turning", false);
            }

            if (keepHeadingSetpointSet && maintainHeading) {
                rotation = keepHeadingPid.calculate(getYaw().getDegrees());
            }
        }

        logger.recordOutput("Swerve/keepHeadingSetpointSet", keepHeadingSetpointSet);
        logger.recordOutput("Swerve/keepSetpoint", keepHeadingPid.getSetpoint());

        SwerveModuleState[] swerveModuleStates =
                KINEMATICS.toSwerveModuleStates(
                        fieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                        translation.getX(), translation.getY(), rotation, getYaw())
                                : new ChassisSpeeds(
                                        translation.getX(), translation.getY(), rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_SPEED);

        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop, false, false);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    // Use in above method?
    public void setModuleStates(
            SwerveModuleState[] desiredStates, boolean openLoop, boolean tuning, boolean parking) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_SPEED);

        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false, tuning, parking);
        }
    }

    public void park() {
        //        for (SwerveModuleState state : getModuleStates()) {
        //            if (state.speedMetersPerSecond > .01) return;
        //        }
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

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public void resetPose(Pose2d pose) {
        swerveOdometry.resetPosition(new Rotation2d(), getModulePositions(), pose);
    }

    public void zeroGyro() {
        gyroIO.zeroGyro();
        keepHeadingSetpointSet = false;
        lastTurnCommandSeconds = Timer.getFPGATimestamp();
    }

    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(gyroInputs.yaw);
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
        logger.processInputs("gyro", gyroInputs);

        swerveOdometry.update(getYaw(), getModulePositions());

        logger.recordOutput("Swerve/Odometry", swerveOdometry.getPoseMeters());
        logger.recordOutput("Swerve/modules", getModuleStates());

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
