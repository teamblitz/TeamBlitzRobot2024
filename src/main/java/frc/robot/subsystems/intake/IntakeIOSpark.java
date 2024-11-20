package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class IntakeIOSpark implements IntakeIO {

    private final SparkMax motor;

    private final DigitalInput breakBeamSensor;

    public IntakeIOSpark() {
        motor = new SparkMax(Constants.Intake.Spark.MOTOR_ID, MotorType.kBrushless);
        breakBeamSensor = new DigitalInput(9);

        motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(Constants.Intake.CURRENT_LIMIT);

        motor.setIdleMode(SparkBaseConfig.IdleMode.kCoast);

        motor.setOpenLoopRampRate(
                0); // TODO: This may have caused the issue of the note going past the sensor
    }

    @Override
    public void set(double speed) {
        motor.set(speed);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.rpm = motor.getEncoder().getVelocity();
        inputs.current = motor.getOutputCurrent();

        inputs.breakBeam = breakBeamSensor.get();
    }
}
