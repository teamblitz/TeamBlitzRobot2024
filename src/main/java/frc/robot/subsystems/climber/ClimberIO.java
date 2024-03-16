package frc.robot.subsystems.climber;
import com.ctre.phoenix6.hardware.TalonFX;


import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    /** All units are meters and radians */
    @AutoLog
    public class ClimberIOInputs {
        public double position;
        public double velocity;


        public double volts;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ClimberIOInputs inputs) {}

    public default void setSpeedLeft(double percent) {}
    public default void setSpeedRight(double percent) {}

    public default void setSetpointLeft(double extension) {}
    public default void setSetpointRight(double extension) {}
    
}
