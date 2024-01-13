package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;

// TODO: Make inlined
public class HoldArmAtPositionCommand extends Command {

    private final ArmSubsystem armSubsystem;

    private double initialPosition;

    public HoldArmAtPositionCommand(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem.rotationRequirement);
    }

    @Override
    public void initialize() {
        initialPosition = armSubsystem.getRotation();
    }

    @Override
    public void execute() {
        armSubsystem.updateRotation(initialPosition, 0);
    }
}
