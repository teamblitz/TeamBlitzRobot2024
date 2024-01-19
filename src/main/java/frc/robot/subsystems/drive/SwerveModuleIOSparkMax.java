package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.CTREConfigs;
import frc.robot.Constants;

public class SwerveModuleIOSparkMax implements SwerveModuleIO {
    private final Rotation2d angleOffset;

    private final CANSparkMax angleMotor;
    private final CANSparkMax driveMotor;
    private final CANcoder absoluteEncoder;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder angleEncoder;

    private final SparkPIDController drivePIDController;
    private final SparkPIDController anglePIDController;

    private boolean lastBrake = false;

    public SwerveModuleIOSparkMax(SwerveModuleConstants moduleConstants) {
        this.angleOffset = moduleConstants.angleOffset;

        /* Absolute Encoder */
        absoluteEncoder = new CANcoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor */
        angleMotor =
                new CANSparkMax(
                        moduleConstants.angleMotorID, MotorType.kBrushless);
        angleEncoder = angleMotor.getEncoder();
        anglePIDController = angleMotor.getPIDController();
        configAngleMotor();

        /* Drive motor */
        driveMotor =
                new CANSparkMax(
                        moduleConstants.driveMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        drivePIDController = driveMotor.getPIDController();
        configDriveMotor();
    }

    @Override
    public void updateInputs(SwerveModuleIO.SwerveModuleInputs inputs) {
        inputs.anglePositionDegrees = angleEncoder.getPosition();
        inputs.drivePositionMeters = driveEncoder.getPosition();
        inputs.speedMetersPerSecond = driveEncoder.getVelocity();
        inputs.absoluteEncoderPositionDegrees = absoluteEncoder.getAbsolutePosition().getValue();
    }

    @Override
    public void setDrivePercent(double percent) {
        driveMotor.set(percent);
    }

    @Override
    public void setDriveSetpoint(double setpoint, double ffVolts) {
        drivePIDController.setReference(
                setpoint,
                ControlType.kVelocity,
                0,
                ffVolts,
                ArbFFUnits.kVoltage);
    }

    @Override
    public void setAngleSetpoint(double setpoint) {
        anglePIDController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void configureDrivePID(double p, double i, double d) {
        drivePIDController.setP(p);
        drivePIDController.setI(i);
        drivePIDController.setD(d);
    }

    @Override
    public void configureAnglePID(double p, double i, double d) {
        anglePIDController.setP(p);
        anglePIDController.setI(i);
        anglePIDController.setD(d);
    }

    private void configAngleEncoder() {
        // absoluteEncoder.
        absoluteEncoder.getConfigurator().apply(CTREConfigs.getInstance().swerveCanCoderConfig);
    }

    private void resetToAbsolute() {
        angleEncoder.setPosition(absoluteEncoder.getAbsolutePosition().getValueAsDouble() - angleOffset.getDegrees());
    }

    private void configAngleMotor() {
        angleMotor.restoreFactoryDefaults();
        angleMotor.setSmartCurrentLimit(Constants.Swerve.ANGLE_SMART_CURRENT_LIMIT);
        angleMotor.setSecondaryCurrentLimit(Constants.Swerve.ANGLE_SECONDARY_CURRENT_LIMIT);
        angleMotor.setInverted(Constants.Swerve.ANGLE_MOTOR_INVERT);
        angleMotor.setIdleMode(Constants.Swerve.ANGLE_NEUTRAL_MODE);

        angleEncoder.setPositionConversionFactor(
                (1 / Constants.Swerve.ANGLE_GEAR_RATIO) // We do 1 over the gear ratio because 1
                        // rotation of the motor is < 1 rotation of
                        // the module
                        * 360); // 1/360 rotations is 1 degree, 1 rotation is 360 degrees.
        resetToAbsolute();

        anglePIDController.setP(Constants.Swerve.ANGLE_KP);
        anglePIDController.setI(Constants.Swerve.ANGLE_KI);
        anglePIDController.setD(Constants.Swerve.ANGLE_KD);
        anglePIDController.setFF(Constants.Swerve.ANGLE_KF);

        // TODO: Adjust this latter after we know the pid loop is not crazy
        // angleMotor.getPIDController().setOutputRange(-.25, .25);
    }

    private void configDriveMotor() {
        driveMotor.restoreFactoryDefaults();
        driveMotor.setSmartCurrentLimit(Constants.Swerve.DRIVE_SMART_CURRENT_LIMIT);
        driveMotor.setSecondaryCurrentLimit(Constants.Swerve.DRIVE_SECONDARY_CURRENT_LIMIT);
        driveMotor.setInverted(Constants.Swerve.DRIVE_MOTOR_INVERT);
        driveMotor.setIdleMode(Constants.Swerve.DRIVE_NEUTRAL_MODE);
        driveMotor.setOpenLoopRampRate(Constants.Swerve.OPEN_LOOP_RAMP);
        driveMotor.setClosedLoopRampRate(Constants.Swerve.CLOSED_LOOP_RAMP);

        driveEncoder.setVelocityConversionFactor(
                1
                        / Constants.Swerve
                                .DRIVE_GEAR_RATIO // 1/gear ratio because the wheel spins slower
                        // than
                        // the motor.
                        * Constants.Swerve
                                .WHEEL_CIRCUMFERENCE // Multiply by the circumference to get meters
                        // per minute
                        / 60); // Divide by 60 to get meters per second.
        driveEncoder.setPositionConversionFactor(
                1 / Constants.Swerve.DRIVE_GEAR_RATIO * Constants.Swerve.WHEEL_CIRCUMFERENCE);
        driveEncoder.setPosition(0);

        drivePIDController.setP(Constants.Swerve.DRIVE_KP);
        drivePIDController.setI(Constants.Swerve.DRIVE_KI);
        drivePIDController.setD(Constants.Swerve.DRIVE_KD);
        drivePIDController.setFF(
                Constants.Swerve
                        .DRIVE_KF); // Not actually used because we specify our feedforward when we
        // set our speed.

        // TODO: Remove after we know the pid loop isn't wild
        // drivePIDController.setOutputRange(-.5, .5);
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        if (lastBrake != enabled) {
            driveMotor.setIdleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
            lastBrake = enabled;
        }
    }
}
