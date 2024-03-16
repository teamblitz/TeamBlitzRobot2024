package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.BlitzSubsystem;
import frc.lib.MutableReference;
import frc.robot.Constants;
import frc.robot.Constants.Arm.FeedForwardConstants;
import frc.robot.Robot;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Maybe divide this into 2 subsystems, depends on how we want to control it. The current way we do
 * this, 2 subsystems is ideal (and is kinda what we are pseudo doing)
 */
public class Arm extends SubsystemBase implements BlitzSubsystem {

    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    private boolean atGoal;

    private final ArmFeedforward feedforward;

    private final SysIdRoutine routine;

    public Arm(ArmIO io) {
        this.io = io;

        feedforward =
                new ArmFeedforward(
                        FeedForwardConstants.KS, FeedForwardConstants.KG, FeedForwardConstants.KV);

        // Do this, but smarter
//        Commands.waitSeconds(5)
//                .andThen(io::seedArmPosition)
//                .andThen(() -> io.setArmSpeed(0)) // Set the arm to 0 to end on board pid
//                // loop
//                .ignoringDisable(true)
//                .schedule();

        // Constructor of armio?
        io.seedArmPosition();

        routine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                Volts.of(5),
                                null,
                                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
                        new SysIdRoutine.Mechanism(
                                (Measure<Voltage> volts) -> {
                                    System.out.println(volts.baseUnitMagnitude());
                                    io.setArmVolts(volts.in(Volts));
                                },
                                null, // No log consumer, since data is recorded by URCL
                                this));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("arm", inputs);
    }

    public void updateRotation(double degrees, double velocity) {
        Logger.recordOutput("arm/wanted_rotation", degrees);
        io.setRotationSetpoint(degrees, feedforward.calculate(degrees, velocity));
    }

    public double getRotation() {
        return inputs.rotation;
    }

    public double getRotationSpeed() {
        return inputs.armRotationSpeed;
    }

    public void setArmRotationSpeed(double percent) {
        io.setArmSpeed(percent);
    }

    public Command rotateToCommand(DoubleSupplier goal, boolean endAutomatically) {
        TrapezoidProfile profile =
                new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(
                                Constants.Arm.ROTATION_VELOCITY,
                                Constants.Arm.ROTATION_ACCELERATION));

        MutableReference<TrapezoidProfile.State> lastState = new MutableReference<>();

        TrapezoidProfile.State goalState = new TrapezoidProfile.State(goal.getAsDouble(), 0);

        return runOnce(
                        () -> {
                            lastState.set(
                                    new TrapezoidProfile.State(
                                            inputs.rotation, inputs.armRotationSpeed));

                            profile.calculate(Robot.defaultPeriodSecs, lastState.get(), goalState);
                        })
                .andThen(
                        run(() -> {
                                    TrapezoidProfile.State setpoint =
                                            profile.calculate(
                                                    Robot.defaultPeriodSecs,
                                                    lastState.get(),
                                                    new TrapezoidProfile.State(
                                                            goal.getAsDouble(), 0));
                                    lastState.set(setpoint);
                                    updateRotation(setpoint.position, setpoint.velocity);
                                    atGoal = profile.timeLeftUntil(goal.getAsDouble()) == 0;
                                })
                                .until(
                                        () ->
                                                profile.timeLeftUntil(goal.getAsDouble()) == 0
                                                        && endAutomatically))
                .finallyDo(
                        (interrupted) -> {
                            if (!interrupted) updateRotation(goal.getAsDouble(), 0);
                            else if (interrupted) updateRotation(lastState.get().position, 0);
                        });
    }

    public Command rotateToCommand(double goal, boolean endAutomatically) {
        return rotateToCommand(() -> goal, endAutomatically);
    }

    public boolean atGoal() {
        return atGoal;
    }

    /* SYSID STUFF */
    // Creates a SysIdRoutine

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

    public Command coastCommand() {
        return Commands.startEnd(() -> io.setBrake(false), () -> io.setBrake(true))
                .ignoringDisable(true);
    }
}
