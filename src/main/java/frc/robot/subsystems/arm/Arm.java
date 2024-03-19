package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.BlitzSubsystem;
import frc.lib.MutableReference;
import frc.lib.util.LoggedTunableNumber;
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
    private final LoggedTunableNumber kP = new LoggedTunableNumber("Arm/kP", Constants.Arm.PidConstants.P);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Arm/kI", Constants.Arm.PidConstants.I);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Arm/kD", Constants.Arm.PidConstants.D);

    private final LoggedTunableNumber kS = new LoggedTunableNumber("Arm/kS", Constants.Arm.FeedForwardConstants.KS);
    private final LoggedTunableNumber kV = new LoggedTunableNumber("Arm/kV", Constants.Arm.FeedForwardConstants.KV);
    private final LoggedTunableNumber kA = new LoggedTunableNumber("Arm/kA", Constants.Arm.FeedForwardConstants.KA);
    private final LoggedTunableNumber kG = new LoggedTunableNumber("Arm/kG", Constants.Arm.FeedForwardConstants.KG);

    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    private boolean atGoal;

    private ArmFeedforward feedforward;

    private final SysIdRoutine routine;

    public Arm(ArmIO io) {
        this.io = io;

        feedforward =
                new ArmFeedforward(
                        FeedForwardConstants.KS, FeedForwardConstants.KG, FeedForwardConstants.KV);

        // Do this, but smarter
        new Trigger(() ->inputs.encoderConnected)
                .onTrue(
                        Commands.waitSeconds(.25)
                                .andThen(io::seedArmPosition)
                                .andThen(() -> io.setArmSpeed(0)) // Set the arm to 0 to end on board pid
                                .ignoringDisable(true));

        new Trigger(DriverStation::isDisabled)
                .whileTrue(
                        run(
                                () -> io.setArmSpeed(0) // Sparks like to go back to where they were before disabled, constantly set the setpoint to avoid this and instead use the trapezoid profile
                        ).ignoringDisable(true)
                );


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

        ShuffleboardTab tab = Shuffleboard.getTab("Sysid");
        tab.add("ArmQuasistaticFwd", sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        tab.add("ArmQuasistaticRev", sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

        tab.add("ArmDynamicFwd", sysIdDynamic(SysIdRoutine.Direction.kForward));
        tab.add("ArmDynamicRev", sysIdDynamic(SysIdRoutine.Direction.kReverse));


        ShuffleboardTab autoShootTab = Shuffleboard.getTab("AutoShoot");
        GenericEntry testArm = autoShootTab.add("testArm", 0).getEntry();
        autoShootTab.add("testArmCmd", this.rotateToCommand(() -> testArm.getDouble(0), false));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("arm", inputs);

        LoggedTunableNumber.ifChanged(hashCode(), pid -> io.setPid(pid[0], pid[1], pid[2]), kP, kI, kD);
        LoggedTunableNumber.ifChanged(
                hashCode(), kSGV -> feedforward = new ArmFeedforward(kSGV[0], kSGV[1], kSGV[2]), kS, kG, kV);
    }

    public void updateRotation(double degrees, double velocity) {
        Logger.recordOutput("arm/wanted_rotation", degrees);
        Logger.recordOutput("arm/wanted_velocity", velocity);
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

    public Command rotateToCommand(DoubleSupplier goal, boolean endAutomatically, boolean restOnEnd) {
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
                        })
                .andThen(
                        startEnd(io::stop, io::stop).onlyIf(() -> restOnEnd)
                )
                .until(DriverStation::isDisabled); // cancel on disable
    }

    public Command rotateToCommand(double goal, boolean endAutomatically, boolean restOnEnd) {
        return rotateToCommand(() -> goal, endAutomatically, restOnEnd);
    }

    public Command rotateToCommand(DoubleSupplier goal, boolean endAutomatically) {
        return rotateToCommand(goal, endAutomatically, false);
    }

    public Command rotateToCommand(double goal, boolean endAutomatically) {
        return rotateToCommand(goal, endAutomatically, false);
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
