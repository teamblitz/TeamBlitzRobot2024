package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.BlitzSubsystem;
// import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase implements BlitzSubsystem {

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    public void shootOpenLoop() {
        io.setPercent(.8); // TODO CONST
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("shooter", inputs);
    }

    public void shootClosedLoop() {
        io.setSetpoint(0); // TODO, CONST
    }

    public void reverse() {
        io.setPercent(-0.3);
    }

    public void stop() {
        io.setPercent(0);
    }

    public Command shootCommand() {
        return startEnd(this::shootOpenLoop, this::stop);
    }

    public Command reverseCommand() {
        return startEnd(this::reverse, this::stop);
    }
}
