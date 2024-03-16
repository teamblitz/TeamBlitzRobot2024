package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    /** All units are meters and radians */
    @AutoLog
    public class ClimberIOInputs {
        public double position;
        public double velocity;

        public boolean topLimitSwitch;
        public boolean bottomLimitSwitch;

        public double volts;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ClimberIOInputs inputs) {}

    public default void setSetpoint(double degrees, double arbFFPercent) {}

    public default void setVelocity(double percent) {}

    public default void setVolts(double volts) {}
    public default void setBrake(boolean brake) {}
}
