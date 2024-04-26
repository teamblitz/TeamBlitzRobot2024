package frc.robot.subsystems.drive.swerveModule.angle;

import com.revrobotics.*;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;

public class AngleMotorIOSpark implements AngleMotorIO {
    private final Rotation2d angleOffset;

    private final CANSparkMax motor;

    private final RelativeEncoder encoder;

    private final SparkPIDController pidController;

    public AngleMotorIOSpark(SwerveModuleConstants moduleConstants) {
        this.angleOffset = moduleConstants.angleOffset;

        /* Angle Motor */
        motor =
                new CANSparkMax(
                        moduleConstants.angleMotorID, CANSparkLowLevel.MotorType.kBrushless);
        encoder = motor.getEncoder();
        pidController = motor.getPIDController();
        configAngleMotor();
    }

    @Override
    public void updateInputs(AngleMotorIO.AngleMotorInputs inputs) {
        inputs.angularVelocity = encoder.getVelocity();
        inputs.rotation = encoder.getPosition();
    }

    @Override
    public void setSetpoint(double setpoint) {
        pidController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void configurePID(double p, double i, double d) {
        pidController.setP(p);
        pidController.setI(i);
        pidController.setD(d);
    }

    @Override
    public void seedPosition(double position) {
        encoder.setPosition(position - angleOffset.getDegrees());
    }

    private void configAngleMotor() {
        motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(Constants.Drive.ANGLE_SMART_CURRENT_LIMIT);
        motor.setSecondaryCurrentLimit(Constants.Drive.ANGLE_SECONDARY_CURRENT_LIMIT);
        motor.setInverted(Constants.Drive.ANGLE_MOTOR_INVERT);
        motor.setIdleMode(
                Constants.Drive.ANGLE_BRAKE_MODE
                        ? CANSparkBase.IdleMode.kBrake
                        : CANSparkBase.IdleMode.kCoast);

        encoder.setPositionConversionFactor(
                (1 / Constants.Drive.ANGLE_GEAR_RATIO) // We do 1 over the gear ratio because 1
                        // rotation of the motor is < 1 rotation of
                        // the module
                        * 360); // 1/360 rotations is 1 degree, 1 rotation is 360 degrees.

        configurePID(Constants.Drive.ANGLE_KP, Constants.Drive.ANGLE_KI, Constants.Drive.ANGLE_KD);
    }
}
