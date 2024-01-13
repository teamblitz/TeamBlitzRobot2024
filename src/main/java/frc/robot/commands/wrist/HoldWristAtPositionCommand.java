package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.wrist.WristSubsystem;

// TODO: Make inlined
public class HoldWristAtPositionCommand extends Command {

    private final WristSubsystem wristSubsystem;

    private double initialPosition;

    public HoldWristAtPositionCommand(WristSubsystem wristSubsystem) {
        this.wristSubsystem = wristSubsystem;
        addRequirements(wristSubsystem);
    }

    @Override
    public void initialize() {
        initialPosition = wristSubsystem.lastGoal;
    }

    @Override
    public void execute() {
        wristSubsystem.updateRotation(initialPosition, 0);
    }
}
