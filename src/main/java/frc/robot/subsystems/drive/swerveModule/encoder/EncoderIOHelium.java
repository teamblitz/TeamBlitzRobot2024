package frc.robot.subsystems.drive.swerveModule.encoder;

import com.reduxrobotics.sensors.canandcoder.Canandcoder;

public class EncoderIOHelium implements EncoderIO {

    private final Canandcoder encoder;

    public EncoderIOHelium(int id, boolean invert) {
        encoder = new Canandcoder(id);

        Canandcoder.Settings settings = new Canandcoder.Settings();
        settings.setInvertDirection(!invert);
        encoder.setSettings(settings);
    }

    @Override
    public void updateInputs(EncoderIO.EncoderIOInputs inputs) {
        inputs.position = encoder.getAbsPosition() * 360;
    }

    public void zeroEncoder() {
        encoder.setPosition(0);
        System.out.println("zeroed");
    }
}