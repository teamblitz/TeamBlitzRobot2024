package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
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
public class Climber extends SubsystemBase implements BlitzSubsystem {

    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private final ElevatorFeedforward feedforward;

    private final SysIdRoutine routine;

    public Climber(ClimberIO io) {
        this.io = io;

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("climber", inputs);
    }

    public void setSpeed(double left, double right) {}

    public double getSpeed() {
        return inputs.armRotationSpeed;
    }

    public Command extendToCommand(double goal, boolean endAutomatically) {
        return extendToCommand(() -> goal, endAutomatically);
    }

}
