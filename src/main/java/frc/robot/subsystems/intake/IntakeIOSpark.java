package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class IntakeIOSpark implements IntakeIO {

    private final CANSparkMax motor;

    private final DigitalInput breakBeamSensor;

    public IntakeIOSpark() {
        motor = new CANSparkMax(Constants.Intake.Spark.MOTOR_ID, MotorType.kBrushless);
        breakBeamSensor = new DigitalInput(0);

        motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(Constants.Intake.CURRENT_LIMIT);

        motor.setOpenLoopRampRate(0.5);
    }

    @Override
    public void set(double speed) {
        System.out.println("Intake: " + speed);
        motor.set(speed);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.rpm = motor.getEncoder().getVelocity();
        inputs.current = motor.getOutputCurrent();

        inputs.breakBeam = breakBeamSensor.get();
    }
}
