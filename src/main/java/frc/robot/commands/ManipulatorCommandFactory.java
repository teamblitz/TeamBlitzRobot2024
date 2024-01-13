package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class ManipulatorCommandFactory {
    private final ArmSubsystem armSubsystem;
    private final WristSubsystem wristSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    public ManipulatorCommandFactory(
            ArmSubsystem armSubsystem,
            IntakeSubsystem intakeSubsystem,
            WristSubsystem wristSubsystem) {
        this.armSubsystem = armSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.wristSubsystem = wristSubsystem;
    }

    /* Arm/Wrist/Manipulator Commands */

    public Command superStructureToState(
            double armRotation, double armExtension, double robotRelativeWristRotation) {
        return armSubsystem
                .rotateToCommand(armRotation)
                .alongWith(Commands.print("Super structure to state"))
                .alongWith(armSubsystem.extendToCommand(armExtension))
                .alongWith(
                        wristSubsystem.rotateToCommand(robotRelativeWristRotation - armRotation));
    }

    public Command primeConeHigh() {
        return superStructureToState(
                Constants.Arm.Position.Rotation.CONE_HIGH,
                Constants.Arm.Position.Extension.CONE_HIGH,
                Constants.Wrist.Position.CONE_HIGH_RELATIVE);
    }

    public Command primeConeMid() {
        return superStructureToState(
                Constants.Arm.Position.Rotation.CONE_MID,
                Constants.Arm.Position.Extension.CONE_MID,
                Constants.Wrist.Position.CONE_MID_RELATIVE);
    }

    public Command primeCubeHigh() {
        return superStructureToState(
                Constants.Arm.Position.Rotation.CUBE_HIGH,
                Constants.Arm.Position.Extension.CUBE_HIGH,
                Constants.Wrist.Position.CUBE_HIGH_RELATIVE);
    }

    public Command primeCubeMid() {
        return superStructureToState(
                Constants.Arm.Position.Rotation.CUBE_MID,
                Constants.Arm.Position.Extension.CUBE_MID,
                Constants.Wrist.Position.CUBE_MID_RELATIVE);
    }

    public Command primeHybrid() {
        return superStructureToState(
                Constants.Arm.Position.Rotation.HYBRID,
                Constants.Arm.Position.Extension.HYBRID,
                Constants.Wrist.Position.HYBRID_ROBOT_RELATIVE);
    }

    public Command primeCubeRamp() {
        return superStructureToState(
                Constants.Arm.Position.Rotation.CUBE_RAMP,
                Constants.Arm.Position.Extension.CUBE_RAMP,
                Constants.Wrist.Position.CUBE_RAMP_ROBOT_RELATIVE);
    }

    public Command primeConeRamp() {
        return superStructureToState(
                Constants.Arm.Position.Rotation.CONE_RAMP,
                Constants.Arm.Position.Extension.CONE_RAMP,
                Constants.Wrist.Position.CONE_RAMP_ROBOT_RELATIVE);
    }

    public Command primeCubeShelf() {
        return superStructureToState(
                Constants.Arm.Position.Rotation.CUBE_SHELF,
                Constants.Arm.Position.Extension.CUBE_SHELF,
                Constants.Wrist.Position.CUBE_SHELF_RELATIVE);
    }

    public Command primeConeShelf() {
        return superStructureToState(
                Constants.Arm.Position.Rotation.CONE_SHELF,
                Constants.Arm.Position.Extension.CONE_SHELF,
                Constants.Wrist.Position.CONE_SHELF_RELATIVE);
    }

    public Command groundCubePickup() {
        return superStructureToState(
                Constants.Arm.Position.Rotation.CUBE_PICKUP_GROUND,
                Constants.Arm.Position.Extension.CUBE_PICKUP_GROUND,
                Constants.Wrist.Position.CUBE_PICKUP_GROUND_ROBOT_RELATIVE);
    }

    public Command groundUprightConePickup() {
        return superStructureToState(
                Constants.Arm.Position.Rotation.CONE_UPRIGHT_PICKUP_GROUND,
                Constants.Arm.Position.Extension.CONE_UPRIGHT_PICKUP_GROUND,
                Constants.Wrist.Position.CONE_UPRIGHT_PICKUP_GROUND_ROBOT_RELATIVE);
    }

    public Command groundFallenConePickup() {
        return superStructureToState(
                Constants.Arm.Position.Rotation.CONE_FALLEN_PICKUP_GROUND,
                Constants.Arm.Position.Extension.CONE_FALLEN_PICKUP_GROUND,
                Constants.Wrist.Position.CONE_FALLEN_PICKUP_GROUND_ROBOT_RELATIVE);
    }
}
