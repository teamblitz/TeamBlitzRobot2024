package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.BlitzSubsystem;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class Shooter extends SubsystemBase implements BlitzSubsystem {

    private final ShooterIO io;
    private final IntakeIOInputs inputs = new IntakeIOInputs();

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    public void shooter() {
        io.set(0.5);
    }

    public void eject() {
        io.set(-0.2);
    }

    public void stop() {
        io.set(0);
    }

    public Command shooterCommand() {
        return startEnd(this::shooter, this::stop);
    }

    public Command ejectCommand() {
        return startEnd(this::eject, this::stop);
    }
}
