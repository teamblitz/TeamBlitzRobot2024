package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.Arm.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.BlitzSubsystem;
import frc.lib.math.EqualsUtil;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.Arm.FeedForwardConstants;
import frc.robot.Robot;
import frc.robot.subsystems.leds.Leds;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import lombok.*;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends BlitzSubsystem {
    private final LoggedTunableNumber kP =
            new LoggedTunableNumber("Arm/kP", Constants.Arm.PidConstants.P);
    private final LoggedTunableNumber kI =
            new LoggedTunableNumber("Arm/kI", Constants.Arm.PidConstants.I);
    private final LoggedTunableNumber kD =
            new LoggedTunableNumber("Arm/kD", Constants.Arm.PidConstants.D);

    private final LoggedTunableNumber kS =
            new LoggedTunableNumber("Arm/kS", Constants.Arm.FeedForwardConstants.KS);
    private final LoggedTunableNumber kV =
            new LoggedTunableNumber("Arm/kV", Constants.Arm.FeedForwardConstants.KV);
    private final LoggedTunableNumber kA =
            new LoggedTunableNumber("Arm/kA", Constants.Arm.FeedForwardConstants.KA);
    private final LoggedTunableNumber kG =
            new LoggedTunableNumber("Arm/kG", Constants.Arm.FeedForwardConstants.KG);

    private static final LoggedTunableNumber transitStage =
            new LoggedTunableNumber("Arm/TransitStage", Positions.TRANSIT_STAGE);
    private static final LoggedTunableNumber transitNormal =
            new LoggedTunableNumber("Arm/TransitNormal", Positions.TRANSIT_NORMAL);

    @NoArgsConstructor(force = true)
    @RequiredArgsConstructor
    public enum Goal {
        INTAKE(new LoggedTunableNumber("Arm/IntakeDegrees", Positions.INTAKE), true),
        CLIMB(new LoggedTunableNumber("Arm/ClimbDegrees", Positions.TRANSIT_STAGE), true),
        AMP(new LoggedTunableNumber("Arm/AmpDegrees", Positions.AMP)),
        SUBWOOFER(new LoggedTunableNumber("Arm/SubwooferDegrees", Positions.SPEAKER_SUB_FRONT)),
        PODIUM(new LoggedTunableNumber("Arm/PodiumDegrees", Positions.SPEAKER_PODIUM)),
        // Used for tuning/debugging, should be unused on field
        CUSTOM(new LoggedTunableNumber("Arm/CustomDegrees", 45)),

        // Non static external state
        TRANSIT(
                arm ->
                        arm.isTransitStage.getAsBoolean()
                                ? transitStage.get()
                                : transitNormal.get()),
        AIM(arm -> Math.toDegrees(arm.aimAngleSupplier.getAsDouble())),

        // Special Cases, must be handled individually by subsystem periodic
        CHARACTERIZING,
        MANUAL;

        private final Function<Arm, Double> armSetpoint;
        private final boolean letRest;

        // Some states require member fields of the subsystem, which do not exist at enum creation.
        private double getRads(Arm arm) {
            return Math.toRadians(armSetpoint != null ? armSetpoint.apply(arm) : Double.NaN);
        }

        // Only a few of our states require member variables of the static, the rest don't need it
        // and are defined as simple double suppliers.
        Goal(DoubleSupplier setpointSupplier) {
            this(setpointSupplier, false);
        }

        Goal(Function<Arm, Double> armSetpoint) {
            this(armSetpoint, false);
        }

        Goal(DoubleSupplier setpointSupplier, boolean letRest) {
            this(x -> setpointSupplier.getAsDouble(), letRest);
        }
    }

    @AutoLogOutput @Getter Goal goal = Goal.TRANSIT;

    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    private final TrapezoidProfile profile =
            new TrapezoidProfile(
                    new TrapezoidProfile.Constraints(
                            Constants.Arm.MAX_VELOCITY, Constants.Arm.MAX_ACCELERATION));
    private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();

    private final DoubleSupplier aimAngleSupplier = () -> 0;
    private final BooleanSupplier isTransitStage = () -> false;

    private ArmFeedforward feedforward;

    private final SysIdRoutine routine;

    public Arm(ArmIO io) {
        super("arm");
        this.io = io;

        setDefaultCommand(setGoal(Goal.TRANSIT));

        feedforward =
                new ArmFeedforward(
                        FeedForwardConstants.KS,
                        FeedForwardConstants.KG,
                        FeedForwardConstants.KV,
                        FeedForwardConstants.KA);

        // Do this, but smarter
        new Trigger(() -> inputs.encoderConnected)
                .onTrue(
                        Commands.waitSeconds(.25)
                                .andThen(() -> io.seedArmPosition(false))
                                .andThen(
                                        () ->
                                                io.setArmSpeed(
                                                        0)) // Set the arm to 0 to end on board pid
                                .ignoringDisable(true));

        new Trigger(DriverStation::isDisabled)
                .whileTrue(
                        run(() -> io.setArmSpeed(0) // Sparks like to go back to where they were
                                // before disabled, constantly set the setpoint
                                // to avoid this and instead use the trapezoid
                                // profile
                                )
                                .ignoringDisable(true));

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
        @SuppressWarnings("resource")
        GenericEntry testArm = autoShootTab.add("testArm", 0).getEntry();
        //        autoShootTab.add(
        //                "testArmCmd",
        //                this.rotateToCommand(() -> Math.toRadians(testArm.getDouble(0)), false));
        // TODO, should be replaced by custom
    }

    @Override
    public void periodic() {
        super.periodic();

        io.updateInputs(inputs);
        Logger.processInputs(logKey, inputs);

        LoggedTunableNumber.ifChanged(
                hashCode(), pid -> io.setPid(pid[0], pid[1], pid[2]), kP, kI, kD);
        LoggedTunableNumber.ifChanged(
                hashCode(),
                kSGVA -> feedforward = new ArmFeedforward(kSGVA[0], kSGVA[1], kSGVA[2], kSGVA[3]),
                kS,
                kG,
                kV,
                kA);

        io.seedArmPosition(false); // TODO, try removing this.

        if (DriverStation.isDisabled()) {
            // Reset profile when disabled
            setpointState = new TrapezoidProfile.State(inputs.rotation, 0);
        }

        if (goal != Goal.MANUAL && goal != Goal.CHARACTERIZING) {
            setpointState =
                    profile.calculate(
                            Robot.defaultPeriodSecs,
                            setpointState,
                            new TrapezoidProfile.State(
                                    MathUtil.clamp(
                                            goal.getRads(this),
                                            Units.degreesToRadians(MIN_ROT),
                                            Units.degreesToRadians(MAX_ROT)),
                                    0));

            if (goal.letRest && EqualsUtil.epsilonEquals(goal.getRads(this), MIN_ROT) && atGoal())
                io.setArmSpeed(0);
            else updateRotation(setpointState.position, setpointState.velocity);
        }
        // TODO, characterization should just work, but manual override still needs implementing
    }

    public void updateRotation(double degrees, double velocity) {
        Logger.recordOutput(logKey + "/wanted_rotation", degrees);
        Logger.recordOutput(logKey + "/wanted_velocity", velocity);
        io.setRotationSetpoint(degrees, feedforward.calculate(degrees, velocity));
    }

    public double getRotation() {
        return inputs.rotation;
    }

    public void setArmRotationSpeed(double percent) {
        io.setArmSpeed(percent);
    }

    public Command setGoal(@NonNull Goal goal) {
        return runEnd(() -> this.goal = goal, () -> this.goal = Goal.TRANSIT)
                .withName("Arm " + goal.toString().toLowerCase());
    }

    @AutoLogOutput
    public boolean atGoal() {
        return EqualsUtil.epsilonEquals(setpointState.position, goal.getRads(this), 1e-3);
    }

    /* SYSID STUFF */
    // Creates a SysIdRoutine
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction)
                .deadlineWith(setGoal(Goal.CHARACTERIZING))
                .withName(
                        logKey
                                + "/quasistatic"
                                + (direction == SysIdRoutine.Direction.kForward ? "Fwd" : "Rev"));
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction)
                .deadlineWith(setGoal(Goal.CHARACTERIZING))
                .withName(
                        logKey
                                + "/dynamic"
                                + (direction == SysIdRoutine.Direction.kForward ? "Fwd" : "Rev"));
    }

    public Command coastCommand() {
        return Commands.startEnd(() -> io.setBrake(false), () -> io.setBrake(true))
                .beforeStarting(() -> Leds.getInstance().armCoast = true)
                .finallyDo(() -> Leds.getInstance().armCoast = false)
                .ignoringDisable(true)
                .withName(logKey + "/coast");
    }
}
