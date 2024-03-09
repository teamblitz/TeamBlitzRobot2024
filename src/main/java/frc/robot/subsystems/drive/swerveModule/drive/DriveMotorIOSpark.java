package frc.robot.subsystems.drive.swerveModule.drive;

import com.revrobotics.*;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;

public class DriveMotorIOSpark implements DriveMotorIO {
    private final CANSparkMax motor;

    private final RelativeEncoder encoder;

    private final SparkPIDController pidController;

    private boolean lastBrake = false;

    public DriveMotorIOSpark(SwerveModuleConstants moduleConstants) {

        /* Drive motor */
        motor =
                new CANSparkMax(
                        moduleConstants.driveMotorID, CANSparkLowLevel.MotorType.kBrushless);
        encoder = motor.getEncoder();
        pidController = motor.getPIDController();
        configDriveMotor();
    }

    @Override
    public void updateInputs(DriveMotorIO.DriveMotorInputs inputs) {
        inputs.position = encoder.getPosition();
        inputs.velocity = encoder.getVelocity();
    }

    @Override
    public void setDrivePercent(double percent) {
        motor.set(percent);
    }

    @Override
    public void setSetpoint(double setpoint, double ffVolts) {
        pidController.setReference(
                setpoint,
                CANSparkBase.ControlType.kVelocity,
                0,
                ffVolts,
                SparkPIDController.ArbFFUnits.kVoltage);
    }

    @Override
    public void configurePID(double p, double i, double d) {
        pidController.setP(p);
        pidController.setI(i);
        pidController.setD(d);
    }

    private void configDriveMotor() {
        motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(Constants.Drive.DRIVE_SMART_CURRENT_LIMIT);
        motor.setSecondaryCurrentLimit(Constants.Drive.DRIVE_SECONDARY_CURRENT_LIMIT);
        motor.setInverted(Constants.Drive.DRIVE_MOTOR_INVERT);
        motor.setIdleMode(Constants.Drive.DRIVE_NEUTRAL_MODE);
        motor.setOpenLoopRampRate(Constants.Drive.OPEN_LOOP_RAMP);
        motor.setClosedLoopRampRate(Constants.Drive.CLOSED_LOOP_RAMP);

        encoder.setVelocityConversionFactor(
                1
                        / Constants.Drive
                                .DRIVE_GEAR_RATIO // 1/gear ratio because the wheel spins slower
                        // than
                        // the motor.
                        * Constants.Drive
                                .WHEEL_CIRCUMFERENCE // Multiply by the circumference to get meters
                        // per minute
                        / 60); // Divide by 60 to get meters per second.
        encoder.setPositionConversionFactor(
                1 / Constants.Drive.DRIVE_GEAR_RATIO * Constants.Drive.WHEEL_CIRCUMFERENCE);
        encoder.setPosition(0);

        configurePID(Constants.Drive.DRIVE_KP, Constants.Drive.DRIVE_KI, Constants.Drive.DRIVE_KD);
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        if (lastBrake != enabled) {
            motor.setIdleMode(
                    enabled ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
            lastBrake = enabled;
        }
    }
}
