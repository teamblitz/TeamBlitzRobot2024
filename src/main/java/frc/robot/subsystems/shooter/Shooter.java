package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.BlitzSubsystem;
// import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase implements BlitzSubsystem {

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private double setpoint;

    private final SysIdRoutine routine;

    public Shooter(ShooterIO io) {
        this.io = io;

        routine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                null,
                                null,
                                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
                        new SysIdRoutine.Mechanism(
                                (volts) -> {
                                    io.setVolts(volts.in(Volts));
                                },
                                null,
                                this));

        ShuffleboardTab tab = Shuffleboard.getTab("Sysid");
        tab.add("ShooterQuasistaticFwd", sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        tab.add("ShooterQuasistaticRev", sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

        tab.add("ShooterDynamicFwd", sysIdDynamic(SysIdRoutine.Direction.kForward));
        tab.add("ShooterDynamicRev", sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    public void shootOpenLoop() {
        io.setPercent(.8); // TODO CONST
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("shooter", inputs);
    }

    public void shootClosedLoop(double metersPerSecond) {
        setpoint = metersPerSecond;
        io.setSetpoint(metersPerSecond); // TODO, CONST
        Logger.recordOutput("shooter/velocitySetpoint");
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

    public Command shootClosedLoopCommand(double metersPerSecond) {
        return startEnd(() -> shootClosedLoop(metersPerSecond), this::stop);
    }

    public boolean atSetpoint() {
        return
                MathUtil.isNear(setpoint, inputs.rpmTop, Constants.Shooter.MAX_VELOCITY * .01) &&
                MathUtil.isNear(setpoint, inputs.rpmBottom, Constants.Shooter.MAX_VELOCITY * .01);
    }


    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }
}
