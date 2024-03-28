package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.BlitzSubsystem;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase implements BlitzSubsystem {

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
        setDefaultCommand(automaticIndex());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("intake", inputs);

        Logger.recordOutput("Intake/State", state.name());
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


    public Command intakeGroundAutomatic(double speed) {
        return setSpeedCommand(speed)
                .until(() -> inputs.breakBeam)
                .andThen(() -> state = State.Unindexed);
    }

    public Command intakeGroundAutomatic() {
        return intakeGroundAutomatic(.7);
    }

    public Command feedShooter() {
        return setSpeedCommand(.15)
                .alongWith(
                        Commands.run(() -> state = State.Empty)
                );
    }


    /**
     * Note, should only after intakeCommandSmart finishes
     */
    public Command indexIntake() {
        return setSpeedCommand(-.07)
                .raceWith(
                        Commands.waitSeconds(0).andThen(
                                Commands.waitUntil(() -> inputs.breakBeam)
                                        .andThen(() -> state = State.Indexed)
                        )).onlyIf(() -> !inputs.breakBeam);
    }

    public Command automaticIndex() {
        return new ConditionalCommand(
                indexIntake(),
                Commands.none(),
                () -> state == State.Unindexed
        );
    }

    public Command ejectCommand() {
        return startEnd(this::eject, this::stop);
    }

    public Command setSpeedCommand(double speed) {
        return startEnd(() -> io.set(speed), this::stop);
    }

    public enum State {
        Indexed, Unindexed, Empty
    }

    private State state = State.Indexed;

    public boolean hasNote() {
        return state != State.Empty; // Todo, current logic kina fails here. We need note shot detection.
    }
}
