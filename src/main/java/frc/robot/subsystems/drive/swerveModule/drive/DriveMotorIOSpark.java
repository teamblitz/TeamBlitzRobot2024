package frc.robot.subsystems.drive.swerveModule.drive;

import com.revrobotics.*;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;

public class DriveMotorIOSpark implements DriveMotorIO {
    private final SparkMax motor;

    private final RelativeEncoder encoder;

    private final SparkClosedLoopController pidController;

    private boolean lastBrake = false;

    public DriveMotorIOSpark(SwerveModuleConstants moduleConstants) {

        /* Drive motor */
        motor = new SparkMax(moduleConstants.driveMotorID, SparkLowLevel.MotorType.kBrushless);
        encoder = motor.getEncoder();
        pidController = motor.getClosedLoopController();
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
                SparkBase.ControlType.kVelocity,
                0,
                ffVolts,
                SparkClosedLoopController.ArbFFUnits.kVoltage);
    }

    @Override
    public void configurePID(double p, double i, double d) {
        pidController.setP(p);
        pidController.setI(i);
        pidController.setD(d);
    }

    private void configDriveMotor() {
        SparkMaxConfig config = new SparkMaxConfig();

        config.smartCurrentLimit(Constants.Drive.CurrentLimits.Spark.DRIVE_SMART_CURRENT_LIMIT)
                .secondaryCurrentLimit(
                        Constants.Drive.CurrentLimits.Spark.DRIVE_SECONDARY_CURRENT_LIMIT)
                .inverted(Constants.Drive.DRIVE_MOTOR_INVERT)
                .idleMode(
                        Constants.Drive.DRIVE_BRAKE_MODE
                                ? SparkBaseConfig.IdleMode.kBrake
                                : SparkBaseConfig.IdleMode.kCoast)
                .openLoopRampRate(Constants.Drive.OPEN_LOOP_RAMP)
                .closedLoopRampRate(Constants.Drive.CLOSED_LOOP_RAMP);

        config.encoder
                .velocityConversionFactor(
                        1
                                / Constants.Drive
                                        .DRIVE_GEAR_RATIO // 1/gear ratio because the wheel spins
                                // slower
                                // than
                                // the motor.
                                * Constants.Drive
                                        .WHEEL_CIRCUMFERENCE // Multiply by the circumference to get
                                // meters
                                // per minute
                                / 60)
                .positionConversionFactor(
                        1 / Constants.Drive.DRIVE_GEAR_RATIO * Constants.Drive.WHEEL_CIRCUMFERENCE);

        configurePID(Constants.Drive.DRIVE_KP, Constants.Drive.DRIVE_KI, Constants.Drive.DRIVE_KD);

        encoder.setPosition(0);
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        if (lastBrake != enabled) {
            motor.setIdleMode(
                    enabled ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);
            lastBrake = enabled;
        }
    }
}
