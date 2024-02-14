package frc.robot.subsystems.drive.swerveModule.encoder;

import org.littletonrobotics.junction.AutoLog;

public interface EncoderIO {
    @AutoLog
    public static class SwerveModuleInputs {
        public double absoluteEncoderPosition;
    }
}
