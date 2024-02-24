package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.BlitzSubsystem;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class Intake extends SubsystemBase implements BlitzSubsystem {

    private final IntakeIO io;
    private final IntakeIOInputs inputs = new IntakeIOInputs();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    public void intake() {
        io.set(0.7);
    }

    public void eject() {
        io.set(-0.3);
    }

    public void stop() {
        io.set(0);
    }

    public Command intakeCommand() {
        return startEnd(this::intake, this::stop);
    }

    public Command ejectCommand() {
        return startEnd(this::eject, this::stop);
    }

    public Command setSpeedCommand(double speed) {
        return startEnd(() -> io.set(speed), this::stop);
    }
}
