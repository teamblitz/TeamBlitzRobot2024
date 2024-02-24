package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

// I want to kill myself, this code is like ahhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhb




// This class is mainly just storage for the autonomous pathing and commands
public class AutonomousPathCommand {
    private final Drive drive;

    private static final List<String> autonomousCommands =
            Arrays.asList(
                    "SquareTest"
                    );

    private static final boolean backupAuto = false;

    public AutonomousPathCommand(final Drive drive) {
        this.drive = drive;
    }

    AutoBuilder



    public Command generateAutonomous(String path) {
        if (backupAuto) {
            return generateBackupAuto(path);
        }

        final List<PathPlannerTrajectory> pathGroup;
        HashMap<String, Command> eventMap = new HashMap<>();
        // This will load the file "FullAuto.path"
        // All paths are in /src/main/deploy/pathplanner
        // Please set robot width/length in PathPlanner to 36.5 x 36.5 inches- > meters (0.9271)

        // Check timeouts! Please test.
        // I'm still treating the prime commands like they do not end.
        // ----- Arm Positions -----
        eventMap.put("armHome", this.armSubsystem.homeArmCommand().withTimeout(3));
        eventMap.put(
                "armConeGround",
                this.manipulatorCommandFactory.groundUprightConePickup().withTimeout(3));
        eventMap.put(
                "armHighCone",
                this.manipulatorCommandFactory
                        .primeConeHigh()
                        .raceWith(intakeSubsystem.slowIntakeCommand())
                        .withTimeout(3));
        eventMap.put("armMidCone", this.manipulatorCommandFactory.primeConeMid().withTimeout(3));
        eventMap.put(
                "armCubeGround", this.manipulatorCommandFactory.groundCubePickup().withTimeout(3));
        eventMap.put(
                "armMidCube",
                this.manipulatorCommandFactory
                        .primeCubeMid()
                        .raceWith(intakeSubsystem.slowIntakeCommand())
                        .withTimeout(3));
        eventMap.put(
                "armHighCube",
                this.manipulatorCommandFactory
                        .primeCubeHigh()
                        .raceWith(intakeSubsystem.slowIntakeCommand())
                        .withTimeout(3));

        // ----- Intake -----
        eventMap.put("intakeCube", this.intakeSubsystem.buildCubeInCommand().withTimeout(1));
        eventMap.put("intakeCone", this.intakeSubsystem.buildConeInCommand().withTimeout(1));

        // ----- Outtake -----
        eventMap.put("outtake", this.intakeSubsystem.buildCubeOutCommand().withTimeout(0.25));

        // ----- Balance -----
        eventMap.put("balance", new AutoBalance(this.drive));
        eventMap.put("buildPark", this.drive.buildParkCommand());

        // ----- Testing -----
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        eventMap.put("marker2", new PrintCommand("Passed marker 2"));

        // Actual Pathing
        pathGroup = PathPlanner.loadPathGroup(path, new PathConstraints(1, .75));

        // Create the AutoBuilder
        SwerveAutoBuilder autoBuilder =
                new SwerveAutoBuilder(
                        // Pose2d supplier
                        this.drive::getPose,
                        // Pose2d consumer (should be resetPose, fix later)
                        this.drive::resetOdometry,
                        // SwerveDriveKinematics
                        Constants.Swerve.KINEMATICS,
                        // Use DrivePID presumably
                        new PIDConstants(Constants.AutoConstants.PX_CONTROLLER, 0, 0),
                        // Use AnglePID presumably
                        new PIDConstants(Constants.AutoConstants.P_THETA_CONTROLLER, 0, 0),
                        // Module states consumer used to output the drive subsystem
                        (states) ->
                                this.drive.setModuleStates(states, false, false, false),
                        eventMap,
                        // Should the path be automatically mirrored depending on alliance color
                        true,
                        this.drive);
        return autoBuilder.fullAuto(pathGroup);
    }

    public Command generateBackupAuto(String path) {
        switch (path) {
            case "Score":
                return manipulatorCommandFactory
                        .primeCubeHigh()
                        .alongWith(intakeSubsystem.slowIntakeCommand())
                        .withTimeout(2)
                        .andThen(intakeSubsystem.buildCubeOutCommand());
            case "Left":
                return autoMidCube()
                        .andThen(() -> System.out.println("Left pre drive"))
                        .andThen(
                                driveOutDistance(2.5).withTimeout(2.5),
                                driveBackDistance(2).withTimeout(2))
                        .andThen(drive.buildParkCommand().repeatedly());
            case "Right":
                return autoMidCube()
                        .andThen(() -> System.out.println("Right pre drive"))
                        .andThen(
                                driveOutDistance(4.5).withTimeout(4.5),
                                driveBackDistance(3).withTimeout(3))
                        .andThen(drive.buildParkCommand().repeatedly());
            case "Middle":
                return autoMidCube()
                        .andThen(driveOutDistance(6).withTimeout(6))
                        .andThen(
                                Commands.run(
                                                () ->
                                                        drive.drive(
                                                                new Translation2d(-1, 0),
                                                                0,
                                                                true,
                                                                true,
                                                                false),
                                                drive)
                                        .until(() -> Math.abs(drive.getPitch()) < -10)
                                        .finallyDo(
                                                (b) ->
                                                        drive.drive(
                                                                new Translation2d(0, 0),
                                                                0,
                                                                true,
                                                                true,
                                                                false))
                                        .andThen(drive.buildParkCommand().repeatedly()));
            case "Balance":
                // return autoMidCube()
                // .andThen(
                //         driveOutDistance(4).withTimeout(4)
                // ).andThen(
                //         Commands.run(
                //                 () -> driveSubsystem.drive(new Translation2d(-1, 0), 0, true,
                // true, false), driveSubsystem
                //         ).until(() -> Math.abs(driveSubsystem.getPitch()) < -10)
                //         .andThen(() -> driveSubsystem.drive(new Translation2d(speedCalculation(),
                // 0), 0, false, true, false))
                // );

                return autoMidCube()
                        .andThen(
                                Commands.run(
                                                () ->
                                                        drive.drive(
                                                                new Translation2d(1, 0),
                                                                0,
                                                                true,
                                                                true,
                                                                false),
                                                drive)
                                        .withTimeout(6))
                        .until(() -> drive.getPitch() > 15)
                        .andThen(
                                Commands.run(
                                                () ->
                                                        drive.drive(
                                                                new Translation2d(.75, 0),
                                                                0,
                                                                true,
                                                                true,
                                                                false),
                                                drive)
                                        .withTimeout(4))
                        .until(() -> drive.getPitch() < 10)
                        .finallyDo(
                                (b) ->
                                        drive.drive(
                                                new Translation2d(0, 0), 0, false, true, false))
                        .andThen(drive.buildParkCommand().repeatedly());
            default:
                return autoMidCube();
        }
    }
}
