package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;

public class ShooterIOSpark implements ShooterIO {

    private final CANSparkMax motor;

    public ShooterIOSpark() {
        motor = new CANSparkMax(Constants.Shooter.Spark.MOTOR_ID, MotorType.kBrushless);

        motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(Constants.Shooter.CURRENT_LIMIT);

        motor.setOpenLoopRampRate(0.5);
    }

    @Override
    public void set(double speed) {
        motor.set(speed);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.rpm = motor.getEncoder().getVelocity();
        inputs.current = motor.getOutputCurrent();
    }
}
