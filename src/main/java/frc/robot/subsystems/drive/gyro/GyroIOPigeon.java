package frc.robot.subsystems.drive.gyro;

import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.Constants;

public class GyroIOPigeon implements GyroIO {

    /*
     * +X points forward.
     * +Y points to the left.
     * +Z points to the sky.
     * Pitch is about +Y
     * Roll is about +X
     * Yaw is about +Z
     *
     * Yaw is Counterclockwise positive.
     * Nose down pitch is positive.
     * Rolling to the right is positive
     *
     * https://store.ctr-electronics.com/content/user-manual/Pigeon2%20User's%20Guide.pdf
     */
    private final Pigeon2 gyro;

    private final double[] rateArray = new double[3];

    public GyroIOPigeon() {
        gyro = new Pigeon2(Constants.Drive.PIGEON_ID);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.yaw = gyro.getYaw().getValueAsDouble();
        inputs.pitch = gyro.getPitch().getValueAsDouble();
        inputs.roll = gyro.getRoll().getValueAsDouble();
        inputs.yawRate = gyro.getAngularVelocityZDevice().getValue();
        inputs.pitchRate = gyro.getAngularVelocityYDevice().getValue();
        inputs.rollRate = gyro.getAngularVelocityXDevice().getValue();
        inputs.connected = gyro.getUpTime().getValueAsDouble() > 0;
    }
}
