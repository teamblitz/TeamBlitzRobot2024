package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    @AutoLog
    public class ShooterIOInputs {
        public double rpm;
        public double current;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void set(double speed) {}
}
