package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    @AutoLog
    public class ShooterIOInputs {
        public double rpmTop;
        public double rpmBottom;
        public double currentTop;
        public double currentBottom;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setPercent(double percent) {}

    public default void setSetpoint(double velocity) {}
}
