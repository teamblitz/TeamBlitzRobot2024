package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

/**
 * Todo, integrate this with constants and swerve module io spark, this class kinda made sense when
 * we copied this with the serve code, but I don't really like that we are using a singleton for it.
 *
 * <p>Unfortunately, although the Config all routine is nice, it doesn't leave much in the way of
 * static constants. We could probably have this all in constants using static {} blocks
 */
public final class CTREConfigs {
    private static CTREConfigs instance;

    public static CTREConfigs getInstance() {
        if (instance == null) {
            instance = new CTREConfigs();
        }
        return instance;
    }

    public final CANcoderConfiguration swerveCanCoderConfig;

    private CTREConfigs() {
        swerveCanCoderConfig = new CANcoderConfiguration();

        // TODO: this sucks, I hate this, this is awful, I don't know why this exists. if this still
        // exists at competition idk.
        swerveCanCoderConfig.MagnetSensor.AbsoluteSensorRange =
                AbsoluteSensorRangeValue.Unsigned_0To1;
        swerveCanCoderConfig.MagnetSensor.SensorDirection =
                Constants.Swerve.CAN_CODER_INVERT
                        ? SensorDirectionValue.Clockwise_Positive
                        : SensorDirectionValue.CounterClockwise_Positive;
    }
}
