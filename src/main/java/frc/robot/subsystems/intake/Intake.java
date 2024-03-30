package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.BlitzSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;

public class Intake extends SubsystemBase implements BlitzSubsystem {

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    BooleanSupplier manualOverride;

    public Intake(IntakeIO io, BooleanSupplier manualOverride) {
        this.io = io;
        this.manualOverride = manualOverride;
        setDefaultCommand(automaticIndex());


        // State updating
        new Trigger(() -> intakeState == IntakeState.Feeding && hasNote())
                .whileTrue(
                        Commands.sequence(
                                Commands.waitUntil(() -> !intakeSensor()),
                                Commands.run(() -> noteState = NoteState.Unindexed),
                                Commands.waitSeconds(.02),
                                Commands.waitUntil(this::intakeSensor),
                                Commands.run(() -> noteState = NoteState.Empty)
                        )
                );
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("intake", inputs);

        Logger.recordOutput("Intake/NoteState", noteState.name());
    }

    private boolean intakeSensor() {
        return inputs.breakBeam;
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
        return Commands.parallel(
                setSpeedCommand(speed)
                        .until(() -> inputs.breakBeam)
                        .andThen(() -> noteState = NoteState.Unindexed),
                Commands.startEnd(
                        () -> intakeState = IntakeState.Intaking,
                        () -> intakeState = IntakeState.Idle
                )
        );
    }

    public Command intakeGroundAutomatic() {
        return intakeGroundAutomatic(.7);
    }

    public Command feedShooter() {
        return Commands.parallel(
                setSpeedCommand(.1),
                Commands.startEnd(
                        () -> intakeState = IntakeState.Feeding,
                        () -> intakeState = IntakeState.Idle
                )
        );
    }


    /**
     * Note, should only after intakeCommandSmart finishes
     */
    public Command indexIntake() {
        return Commands.parallel(
                setSpeedCommand(-.07).raceWith(
                        Commands.waitUntil(() -> inputs.breakBeam)
                                .andThen(() -> noteState = NoteState.Indexed)),
                Commands.startEnd(
                        () -> intakeState = IntakeState.Indexing,
                        () -> intakeState = IntakeState.Idle
                )
        ).onlyIf(() -> !inputs.breakBeam);
//        return setSpeedCommand(-.07)
//
//                        );
    }

    public Command automaticIndex() {
        return new ConditionalCommand(
                indexIntake(),
                Commands.none(),
                () -> noteState == NoteState.Unindexed
        );
    }

    public Command ejectCommand() {
        return Commands.parallel(
                setSpeedCommand(-.3),
                Commands.startEnd(
                        () -> intakeState = IntakeState.Ejecting,
                        () -> intakeState = IntakeState.Idle
                )
        );
    }

    public Command setSpeedCommand(double speed) {
        return startEnd(() -> io.set(speed), this::stop);
    }

    public enum NoteState {
        Indexed, Unindexed, Empty, Unknown
    }

    private NoteState noteState = NoteState.Indexed; // We start the match with the note in an indexed state
    
    public enum IntakeState {
        Intaking, Feeding, Ejecting, Indexing, Idle, Manual
    }

    private IntakeState intakeState = IntakeState.Idle;

    public boolean hasNote() {
        return noteState == NoteState.Indexed || noteState == NoteState.Unindexed;
    }
}
