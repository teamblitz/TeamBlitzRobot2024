package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    /** All units are meters and radians */
    @AutoLog
    public class ArmIOInputs {
        public double rotation;
        public double armRotationSpeed;
        public double absArmRot;

        public double absArmEncoder;

        public boolean topRotationLimit;
        public boolean bottomRotationLimit;

        public boolean encoderConnected;

        public double volts;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ArmIOInputs inputs) {}

    public default void setRotationSetpoint(double degrees, double arbFFPercent) {}

    public default void setArmSpeed(double percent) {}

    public default void setArmVolts(double volts) {}

    public default void seedArmPosition() {}

    public default void checkLimitSwitches() {}
}
