package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.wrist.WristSubsystem;

// TODO: Make inlined
public class HoldWristAtRelativePositionCommand extends Command {

    private final WristSubsystem wristSubsystem;

    private double initialPosition;

    public HoldWristAtRelativePositionCommand(WristSubsystem wristSubsystem) {
        this.wristSubsystem = wristSubsystem;

        addRequirements(wristSubsystem);
    }

    @Override
    public void initialize() {
        initialPosition = wristSubsystem.lastRelativeGoal;
    }

    @Override
    public void execute() {
        wristSubsystem.updateRelativeRotation(initialPosition, 0);
    }
}
