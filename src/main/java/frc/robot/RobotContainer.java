/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SwerveModuleIO;
import frc.robot.subsystems.drive.SwerveModuleIOSparkMax;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIONavx;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSpark;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOTalon;

import frc.robot.Constants.Shooter.Spark;
import frc.robot.Constants.Shooter.Talon;


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

    /* ***** --- Controllers --- ***** */
    private Controller controller;
    //     private SaitekX52Joystick driveController;
    private Joystick driveController;

    // private final Logger logger = Logger.getInstance();

    /* ***** --- Autonomous --- ***** */
    // *** Must match with path names in pathplanner folder ***
    private static final String[] autonomousCommands = {
        "Left1sM",
        "Left2sB",
        "Left2sHM",
        "Left2sMB",
        "MiddleL1sB",
        "MiddleL1sMB",
        "MiddleR1sB",
        "MiddleR1sMB",
        "MiddleC1sMB",
        "Right1sM",
        "Right2sHM",
        "Right2sMB",
        "Score",
        "SquareTest",
        "BalanceTest",
        "Nothing"
    };
    private final SendableChooser<String> chooser = new SendableChooser<>();

    public RobotContainer() {
        configureSubsystems();

        configureButtonBindings();
        setDefaultCommands();

        //        CameraServer.startAutomaticCapture();

        DriverStation.silenceJoystickConnectionWarning(true);
        Shuffleboard.getTab("Drive")
                .add("ResetOdometry", Commands.runOnce(() -> drive.resetOdometry(new Pose2d())));

        for (String auto : autonomousCommands) {
            chooser.addOption(auto, auto);
        }
        chooser.setDefaultOption("Nothing", "Nothing");
        SmartDashboard.putData("Autonomous Choices", chooser);
    }

    private void setDefaultCommands() {
        drive.setDefaultCommand(
                new TeleopSwerve(
                        drive,
                        () ->
                                OIConstants.inputCurve.apply(
                                        -driveController.getY() * calculateDriveMultiplier()),
                        () ->
                                OIConstants.inputCurve.apply(
                                        -driveController.getX() * calculateDriveMultiplier()),
                        () -> OIConstants.inputCurve.apply(-driveController.getTwist()) * .3,
                        () -> false,
                        () -> driveController.getPOV()));
    }

    private final SlewRateLimiter driveMultiplierLimiter = new SlewRateLimiter(.25);

    private double calculateDriveMultiplier() {
        if (driveController.getRawButton(2)) {
            return driveMultiplierLimiter.calculate(.3);
        } else if (driveController.getRawButton(1)) {
            return driveMultiplierLimiter.calculate(1);
        } else {
            return driveMultiplierLimiter.calculate(.60);
        }
    }

    private void configureSubsystems() {
        // drive =
        //         new Drive(
        //                 new SwerveModuleIOSparkMax(Constants.Swerve.Mod0.CONSTANTS),
        //                 new SwerveModuleIOSparkMax(Constants.Swerve.Mod1.CONSTANTS),
        //                 new SwerveModuleIOSparkMax(Constants.Swerve.Mod2.CONSTANTS),
        //                 new SwerveModuleIOSparkMax(Constants.Swerve.Mod3.CONSTANTS),
        //                 Constants.Swerve.USE_PIGEON ? new GyroIOPigeon() : new GyroIONavx());

        drive = new Drive(
            new SwerveModuleIO() {},
            new SwerveModuleIO() {},
            new SwerveModuleIO() {},
            new SwerveModuleIO() {},
            new GyroIO() {}
        );

        intake = new Intake(new IntakeIOSpark());
        shooter = new Shooter(new ShooterIOTalon());

        driveController = new Joystick(0); // Move this to Controller (and I never did)
        controller = new Controller(0, 1);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        controller.restGyroTrigger().onTrue(Commands.runOnce(drive::zeroGyro));
        controller.xBrakeTrigger().onTrue(drive.buildParkCommand());

        controller.brakeModeTrigger().onTrue(Commands.runOnce(() -> drive.setBrakeMode(true)));
        controller.coastModeTrigger().onTrue(Commands.runOnce(() -> drive.setBrakeMode(false)));

        controller.getStartTrigger().whileTrue(drive.driveSpeedTestCommand(1, 4));
        controller.getBackTrigger().whileTrue(drive.driveSpeedTestCommand(-1, 4));

        OIConstants.intake.whileTrue(intake.intakeCommand());
        OIConstants.shooter.whileTrue(shooter.shootCommand());
    }

    public Command getAutonomousCommand() { // Autonomous code goes here
        String autoCommand = chooser.getSelected();
        //     AutonomousPathCommand autonomousPathCommand =
        //             new AutonomousPathCommand(
        //                     drive, armSubsystem, intakeSubsystem,
        // manipulatorCommandFactory);
        //     return autonomousPathCommand.generateAutonomous(autoCommand);
        return null;
    }
}
