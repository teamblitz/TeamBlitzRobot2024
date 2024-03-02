package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.BlitzSubsystem;
// import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class Shooter extends SubsystemBase implements BlitzSubsystem {

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

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

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

}
