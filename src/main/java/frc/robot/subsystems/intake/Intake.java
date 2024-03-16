package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.BlitzSubsystem;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase implements BlitzSubsystem {

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("intake", inputs);
    }

    public void intake() {
        io.set(0.7);
    } // was .7

    public void eject() {
        io.set(-0.3);
    }

    public void stop() {
        io.set(0);
    }

    public Command intakeCommand() {
        return startEnd(this::intake, this::stop);
    }

    public Command intakeCommandSmart() {
        return intakeCommand().until(() -> inputs.breakBeam);
    }

    public Command intakeCommandSmart(double speed) {
        return setSpeedCommand(speed).until(() -> inputs.breakBeam);
    }

    /**
     * Note, should only after intakeCommandSmart finishes
     */
    public Command indexIntake() {
        return setSpeedCommand(-.07)
                .raceWith(Commands.waitSeconds(.2).andThen(Commands.waitUntil(() -> inputs.breakBeam)));
    }

    public Command ejectCommand() {
        return startEnd(this::eject, this::stop);
    }

    public Command setSpeedCommand(double speed) {
        return startEnd(() -> io.set(speed), this::stop);
    }
}
