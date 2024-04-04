/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.InternalButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.MutableReference;
import frc.lib.util.LimelightHelpers;
import frc.robot.Constants.AutoConstants.StartingPos;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOSpark;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOKraken;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIONavx;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon;
import frc.robot.subsystems.drive.swerveModule.SwerveModule;
import frc.robot.subsystems.drive.swerveModule.SwerveModuleConfiguration;
import frc.robot.subsystems.drive.swerveModule.angle.AngleMotorIOSim;
import frc.robot.subsystems.drive.swerveModule.drive.DriveMotorIOSim;
import frc.robot.subsystems.drive.swerveModule.encoder.EncoderIO;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSpark;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSpark;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* ***** --- Subsystems --- ***** */
    private Drive drive;
    private Intake intake;
    private Shooter shooter;
    private Arm arm;
    private Climber climber;
    private Leds leds = Leds.getInstance();

    /* ***** --- Shared Commands --- ***** */
    public Command autoShootSpeed;

    /* ***** --- Autonomous --- ***** */
    // *** Must match with path names in pathplanner folder ***
    private final LoggedDashboardChooser<Command> autoChooser;

    private final LoggedDashboardChooser<Constants.AutoConstants.StartingPos>
            startingPositionChooser;

    public RobotContainer() {
        Leds.getInstance();

        configureSubsystems();

        configureButtonBindings();
        setDefaultCommands();
        configureAutoCommands();

        DriverStation.silenceJoystickConnectionWarning(true);
        Shuffleboard.getTab("Drive")
                .add("ResetOdometry", Commands.runOnce(() -> drive.resetOdometry(new Pose2d())));

        autoChooser = new LoggedDashboardChooser<>("autoChoice", AutoBuilder.buildAutoChooser());

        startingPositionChooser = new LoggedDashboardChooser<>("startingPos");
        startingPositionChooser.addDefaultOption("Center", StartingPos.CENTER);
        startingPositionChooser.addOption("Left", StartingPos.LEFT);
        startingPositionChooser.addOption("Right", StartingPos.RIGHT);

        Shuffleboard.getTab("AutoShoot")
                .addDouble(
                        "distance",
                        () ->
                                AutoAimCalculator.calculateDistanceToGoal(
                                        new Pose3d(drive.getEstimatedPose())));

        Shuffleboard.getTab("AutoShoot")
                .addDouble(
                        "speed",
                        () ->
                                AutoAimCalculator.calculateShooterSpeedInterpolation(
                                        AutoAimCalculator.calculateDistanceToGoal(
                                                new Pose3d(drive.getEstimatedPose()))));
    }

    private void setDefaultCommands() {
        drive.setDefaultCommand(
                new TeleopSwerve(
                                drive,
                                OIConstants.Drive.X_TRANSLATION,
                                OIConstants.Drive.Y_TRANSLATION,
                                OIConstants.Drive.ROTATION_SPEED,
                                () -> false,
                                () ->
                                        OIConstants.Drive.AIM_SPEAKER.getAsBoolean()
                                                ? AutoAimCalculator.calculateSpeakerHeading(
                                                                drive.getEstimatedPose())
                                                        .getDegrees()
                                                : Double.NaN)
                        .withName("TeleopSwerve"));

        arm.setDefaultCommand(
                arm.rotateToCommand(
                                () ->
                                        (OIConstants.Arm.TRANSIT_STAGE.getAsBoolean()
                                                ? Constants.Arm.Positions.TRANSIT_STAGE
                                                : Constants.Arm.Positions.TRANSIT_NORMAL),
                                false)
                        .withName("arm/transitPosition"));

        new Trigger(() -> Math.abs(OIConstants.Arm.MANUAL_ARM_SPEED.getAsDouble()) > .08)
                .whileTrue(
                        Commands.run(
                                        () -> {
                                            arm.setArmRotationSpeed(
                                                    OIConstants.Arm.MANUAL_ARM_SPEED.getAsDouble());
                                        },
                                        arm)
                                .finallyDo(() -> arm.setArmRotationSpeed(0))
                                .withName("arm/manual"));
    }

    private void configureSubsystems() {

        drive =
                switch (Constants.robot) {
                    case CompBot -> new Drive(
                            new SwerveModuleConfiguration(
                                    SwerveModuleConfiguration.MotorType.KRAKEN,
                                    SwerveModuleConfiguration.MotorType.NEO,
                                    SwerveModuleConfiguration.EncoderType.CANCODER),
                            Constants.Drive.Mod0.CONSTANTS,
                            Constants.Drive.Mod1.CONSTANTS,
                            Constants.Drive.Mod2.CONSTANTS,
                            Constants.Drive.Mod3.CONSTANTS,
                            Constants.Drive.USE_PIGEON ? new GyroIOPigeon() : new GyroIONavx());

                    case DevBot -> new Drive(
                            new SwerveModuleConfiguration(
                                    SwerveModuleConfiguration.MotorType.NEO,
                                    SwerveModuleConfiguration.MotorType.NEO,
                                    SwerveModuleConfiguration.EncoderType.HELIUM),
                            Constants.Drive.Mod0.CONSTANTS,
                            Constants.Drive.Mod1.CONSTANTS,
                            Constants.Drive.Mod2.CONSTANTS,
                            Constants.Drive.Mod3.CONSTANTS,
                            Constants.Drive.USE_PIGEON ? new GyroIOPigeon() : new GyroIONavx());
                    case SimBot -> new Drive(
                            new SwerveModule(
                                    Constants.Drive.FL,
                                    new AngleMotorIOSim(),
                                    new DriveMotorIOSim(),
                                    new EncoderIO() {}),
                            new SwerveModule(
                                    Constants.Drive.FR,
                                    new AngleMotorIOSim(),
                                    new DriveMotorIOSim(),
                                    new EncoderIO() {}),
                            new SwerveModule(
                                    Constants.Drive.BL,
                                    new AngleMotorIOSim(),
                                    new DriveMotorIOSim(),
                                    new EncoderIO() {}),
                            new SwerveModule(
                                    Constants.Drive.BR,
                                    new AngleMotorIOSim(),
                                    new DriveMotorIOSim(),
                                    new EncoderIO() {}),
                            new GyroIO() {});
                };

        intake = new Intake(new IntakeIOSpark(), OIConstants.Overrides.INTAKE_OVERRIDE);
        shooter = new Shooter(new ShooterIOSpark());
        arm = new Arm(new ArmIOSpark(true));
        climber = new Climber(Constants.compBot() ? new ClimberIOKraken() {} : new ClimberIO() {});

        autoShootSpeed =
                shooter.shootClosedLoopCommand(
                        () ->
                                AutoAimCalculator.calculateShooterSpeedInterpolation(
                                        AutoAimCalculator.calculateDistanceToGoal(
                                                new Pose3d(drive.getLimelightPose()))));
    }

    private void configureButtonBindings() {

        OIConstants.Drive.RESET_GYRO.onTrue(Commands.runOnce(drive::zeroGyro));
        OIConstants.Drive.X_BREAK.onTrue(drive.buildParkCommand());

        OIConstants.Drive.BRAKE.onTrue(Commands.runOnce(() -> drive.setBrakeMode(true)));
        OIConstants.Drive.COAST.onTrue(Commands.runOnce(() -> drive.setBrakeMode(false)));

        NetworkTableEntry intakeTx =
                LimelightHelpers.getLimelightNTTableEntry("limelight-intake", "tx");
        NetworkTableEntry intakeTv =
                LimelightHelpers.getLimelightNTTableEntry("limelight-intake", "tv");

        Debouncer tvBouncer = new Debouncer(2. / 30., Debouncer.DebounceType.kBoth);
        MutableReference<Double> txCache = new MutableReference<>(0.);
        MutableReference<Boolean> tvCache = new MutableReference<>(false);

        OIConstants.Drive.AUTO_PICKUP.whileTrue(
                drive.chaseVector(
                                () ->
                                        new Translation2d(
                                                        Math.cos(
                                                                Math.toRadians(
                                                                        -txCache.get() * 1.7)),
                                                        Math.sin(
                                                                Math.toRadians(
                                                                        -txCache.get() * 1.7)))
                                                .rotateBy(drive.getYaw()),
                                () -> -txCache.get(),
                                3,
                                6)
                        .until(() -> !tvCache.get())
                        .beforeStarting(() -> Leds.getInstance().autoPickupActive = true)
                        .finallyDo(() -> Leds.getInstance().autoPickupActive = false)
                        .onlyIf(() -> tvCache.get()));

        Commands.run(
                        () -> {
                            tvCache.set(tvBouncer.calculate(intakeTv.getDouble(0) == 1));
                            if (intakeTv.getDouble(0) == 1) {
                                txCache.set(intakeTx.getDouble(0));
                            }
                        })
                .ignoringDisable(true)
                .schedule();

        new Trigger(() -> tvBouncer.calculate(intakeTv.getDouble(0) == 1))
                .whileTrue(
                        Commands.startEnd(
                                        () -> Leds.getInstance().autoPickupReady = true,
                                        () -> Leds.getInstance().autoPickupReady = false)
                                .ignoringDisable(true));

        OIConstants.Intake.FEED.whileTrue(intake.feedShooter());
        OIConstants.Intake.EJECT.whileTrue(intake.ejectCommand());
        OIConstants.Shooter.MANUAL_FEED.whileTrue(shooter.shootCommand());
        OIConstants.Shooter.SHOOTER_AMP.whileTrue(shooter.shootCommand());
        OIConstants.Shooter.EJECT.whileTrue(shooter.reverseCommand());
        OIConstants.Shooter.SPEED_AUTO.whileTrue(autoShootSpeed);

        OIConstants.Arm.INTAKE.whileTrue(
                arm.rotateToCommand(Constants.Arm.Positions.INTAKE, true, true)
                        .raceWith(
                                intake.intakeGroundAutomatic()
                                        .raceWith(shooter.setSpeedCommand(-.1))));
        // OIConstants.Arm.TRANSIT_STAGE.whileTrue(
        //         arm.rotateToCommand(Constants.Arm.Positions.TRANSIT_STAGE, false));

        OIConstants.Arm.SPEAKER_SUB_FRONT.whileTrue(
                arm.rotateToCommand(Constants.Arm.Positions.SPEAKER_SUB_FRONT, false)
                        .alongWith(shooter.shootCommand()));

        OIConstants.Arm.SPEAKER_SUB_SIDE.whileTrue(
                arm.rotateToCommand(Constants.Arm.Positions.SPEAKER_SUB_SIDE, false)
                        .alongWith(shooter.shootCommand()));

        OIConstants.Arm.SPEAKER_PODIUM.whileTrue(
                arm.rotateToCommand(Constants.Arm.Positions.SPEAKER_PODIUM, false)
                        .alongWith(shooter.shootCommand()));

        OIConstants.Arm.SCORE_AMP.whileTrue(
                arm.rotateToCommand(Constants.Arm.Positions.SCORE_AMP, false));

        OIConstants.Arm.AUTO_AIM_SPEAKER.whileTrue(buildAutoShootCommand());

        // CLIMBER COMMANDS

        // OIConstants.Climber.UP_BOTH.whileTrue(climber.setSpeed(0.3, 0.3));
        OIConstants.Climber.UP_BOTH.whileTrue(climber.goUp());
        //        OIConstants.Climber.UP_LEFT.whileTrue(climber.setSpeed(0.3, 0));
        //        OIConstants.Climber.UP_RIGHT.whileTrue(climber.setSpeed(0, 0.3));

        // OIConstants.Climber.DOWN_BOTH.whileTrue(climber.setSpeed(-0.3, -0.3));

        OIConstants.Climber.DOWN_BOTH.whileTrue(climber.climb());
        OIConstants.Climber.DOWN_MAN.whileTrue(climber.setSpeed(-0.3, -0.3));
        //        OIConstants.Climber.DOWN_RIGHT.whileTrue(climber.setSpeed(0, -0.3));

        OIConstants.Climber.DOWN_BOTH.onTrue(
                arm.rotateToCommand(Constants.Arm.Positions.TRANSIT_STAGE, false).repeatedly());

        OIConstants.Climber.UP_BOTH.onTrue(
                arm.rotateToCommand(Constants.Arm.Positions.TRANSIT_STAGE, false).repeatedly());

        // TEST STUFF
        OIConstants.TestMode.zeroAbsEncoders.onTrue(drive.zeroAbsEncoders());
        OIConstants.TestMode.SysId.Arm.quasistaticFwd.whileTrue(
                arm.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
                        .beforeStarting(Commands.print("ArmQuasFwd")));
        OIConstants.TestMode.SysId.Arm.quasistaticRev.whileTrue(
                arm.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
                        .beforeStarting(Commands.print("ArmQuasRev")));
        OIConstants.TestMode.SysId.Arm.dynamicFwd.whileTrue(
                arm.sysIdDynamic(SysIdRoutine.Direction.kForward)
                        .beforeStarting(Commands.print("ArmDynamicFwd")));
        OIConstants.TestMode.SysId.Arm.dynamicRev.whileTrue(
                arm.sysIdDynamic(SysIdRoutine.Direction.kReverse)
                        .beforeStarting(Commands.print("ArmDynamicRev")));

        OIConstants.TestMode.SysId.Drive.quasistaticFwd.whileTrue(
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
                        .beforeStarting(Commands.print("DriveQuasFwd")));
        OIConstants.TestMode.SysId.Drive.quasistaticRev.whileTrue(
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
                        .beforeStarting(Commands.print("DriveQuasRev")));
        OIConstants.TestMode.SysId.Drive.dynamicFwd.whileTrue(
                drive.sysIdDynamic(SysIdRoutine.Direction.kForward)
                        .beforeStarting(Commands.print("DriveDynamicFwd")));
        OIConstants.TestMode.SysId.Drive.dynamicRev.whileTrue(
                drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)
                        .beforeStarting(Commands.print("DriveDynamicRev")));

        new Trigger(RobotController::getUserButton).toggleOnTrue(arm.coastCommand());
    }

    private void configureAutoCommands() {

        InternalButton readyShoot = new InternalButton();
        InternalButton shoot = new InternalButton();
        InternalButton qShoot = new InternalButton();

        shoot.whileTrue(
                arm.rotateToCommand(
                                Constants.Arm.Positions.SPEAKER_SUB_FRONT
                                        + Units.degreesToRadians(2),
                                false)
                        .raceWith(Commands.waitSeconds(1))
                        .andThen(intake.feedShooter().asProxy().withTimeout(.5))
                        .raceWith(shooter.shootCommand())
                        .asProxy()
                        .withName("auto/shoot"));

        qShoot.whileTrue(
                arm.rotateToCommand(
                                Constants.Arm.Positions.SPEAKER_SUB_FRONT
                                        + Units.degreesToRadians(2),
                                false)
                                .alongWith(
                                        intake.feedShooter(.7).asProxy()
                                ).raceWith(shooter.shootCommand()).asProxy()
                                .withTimeout(.75)
                        .withName("auto/qShoot"));

        readyShoot.whileTrue(
                arm.rotateToCommand(
                                Constants.Arm.Positions.SPEAKER_SUB_FRONT
                                        + Units.degreesToRadians(2),
                                false)
                        .asProxy()
                        .alongWith(shooter.shootCommand().asProxy())
                        .asProxy()
                        .withName("auto/readyShoot"));
        // Does end
        NamedCommands.registerCommand(
                "shoot",
                Commands.runOnce(
                                () -> {
                                    shoot.setPressed(true);
                                    qShoot.setPressed(false);
                                    readyShoot.setPressed(false);
                                })
                        .andThen(Commands.waitSeconds(1.5)));

        NamedCommands.registerCommand(
                "qshoot",
                Commands.runOnce(
                                () -> {
                                    shoot.setPressed(false);
                                    qShoot.setPressed(true);
                                    readyShoot.setPressed(false);
                                })
                        .andThen(Commands.waitSeconds(.75)));

        //        NamedCommands.registerCommand(
        //                "cShoot",
        //                arm.rotateToCommand(Constants.Arm.Positions.SPEAKER_SUB_SIDE, false)
        //                        .raceWith(Commands.waitSeconds(1))
        //                        .andThen(intake.feedShooter().asProxy().withTimeout(.5))
        //                        .raceWith(shooter.shootCommand()));

        //        NamedCommands.registerCommand(
        //                "autoShoot",
        //                buildAutoShootCommand()
        //                        .raceWith(Commands.waitSeconds(1.5)
        //
        // .andThen(intake.feedShooter().asProxy().withTimeout(.5)))
        //                //
        //                // Commands.waitSeconds(2).andThen(intake.intakeCommand().withTimeout(.5))
        //                //
        // .raceWith(todoPutThisAutoShootSomewhereElse())
        //                );

        // Does not end
        NamedCommands.registerCommand(
                "intake",
                arm.rotateToCommand(Constants.Arm.Positions.INTAKE, true, true)
                        .asProxy()
                        .alongWith(intake.intakeGroundAutomatic(.7).asProxy())
                        .alongWith(shooter.setSpeedCommand(-.1).asProxy()));
        //        NamedCommands.registerCommand(
        //                "index",
        //                intake.indexIntake()
        //        );

        NamedCommands.registerCommand(
                "readyShoot",
                Commands.runOnce(
                                () -> {
                                    System.out.println("READY SHOOT");
                                    shoot.setPressed(false);
                                    qShoot.setPressed(false);
                                    readyShoot.setPressed(true);
                                })
                        .andThen(Commands.waitSeconds(1)));
    }

    public Command getAutonomousCommand() { // Autonomous code goes here
        return Commands.runOnce(() -> drive.setGyro(startingPositionChooser.get().angle))
                .andThen(autoChooser.get().asProxy());
    }

    //    public Command todoPutThisAutoShootSomewhereElse() {
    //        return arm.rotateToCommand(
    //                        () ->
    //                                MathUtil.clamp(
    //                                        AutoAimCalculator.calculateArmAngle(
    //                                                new Pose3d(drive.getLimelightPose()),
    //                                                DriverStation.getAlliance().isPresent()
    //                                                                &&
    // DriverStation.getAlliance().get()
    //                                                                        ==
    // DriverStation.Alliance
    //                                                                                .Blue
    //                                                        ? Constants.Shooter.AutoShootConstants
    //                                                                .goalPoseBlue
    //                                                        : Constants.Shooter.AutoShootConstants
    //                                                                .goalPoseRed,
    //                                                Constants.Shooter.AutoShootConstants
    //                                                        .botToCenterOfRotation,
    //                                                Constants.Shooter.AutoShootConstants
    //                                                        .centerOfRotationToShooter,
    //                                                Constants.Shooter.AutoShootConstants
    //                                                        .shootAngleOffset,
    //
    // Constants.Shooter.AutoShootConstants.shootVelocity),
    //                                        0,
    //                                        Math.PI / 2),
    //                        false)
    //                .alongWith(
    //                        shooter.shootClosedLoopCommand(
    //                                Constants.Shooter.AutoShootConstants.shootVelocity));
    //    }

    public Command buildAutoShootCommand() {
        return arm.rotateToCommand(
                        () ->
                                MathUtil.clamp(
                                        AutoAimCalculator.calculateArmAngleInterpolation(
                                                AutoAimCalculator.calculateDistanceToGoal(
                                                        new Pose3d(drive.getEstimatedPose()))),
                                        0,
                                        Math.PI / 2),
                        false)
                .alongWith(
                        shooter.shootClosedLoopCommand(
                                () ->
                                        AutoAimCalculator.calculateShooterSpeedInterpolation(
                                                AutoAimCalculator.calculateDistanceToGoal(
                                                        new Pose3d(drive.getLimelightPose())))));
    }
}
