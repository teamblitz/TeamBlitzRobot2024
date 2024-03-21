package frc.robot.subsystems.arm;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.math.Angles;
import frc.robot.Constants;
import frc.robot.Constants.Arm;

public class ArmIOSpark implements ArmIO {

    private final CANSparkMax armRotLeader;
    private final CANSparkMax armRotFollower;
    private final RelativeEncoder angleEncoder;
    private final SparkPIDController anglePid;
    private final DutyCycleEncoder absRotationEncoder;

    private final DigitalInput armTopLimitSwitch;
    private final DigitalInput armBottomLimitSwitch;

    public ArmIOSpark() {
        /* Arm Rotation */
        armRotLeader = new CANSparkMax(Arm.ARM_ROT_LEADER, CANSparkLowLevel.MotorType.kBrushless);
        armRotFollower =
                new CANSparkMax(Arm.ARM_ROT_FOLLOWER, CANSparkLowLevel.MotorType.kBrushless);

        armRotLeader.restoreFactoryDefaults();
        armRotFollower.restoreFactoryDefaults();

        armRotLeader.setIdleMode(IdleMode.kBrake);
        armRotFollower.setIdleMode(IdleMode.kBrake);

        armRotLeader.setOpenLoopRampRate(Arm.OPEN_LOOP_RAMP);
        armRotFollower.setOpenLoopRampRate(Arm.OPEN_LOOP_RAMP);

        armRotLeader.setSmartCurrentLimit(Arm.CURRENT_LIMIT);
        armRotLeader.setSmartCurrentLimit(Arm.CURRENT_LIMIT);

        armRotLeader.setInverted(false);
        armRotFollower.follow(armRotLeader, true);

        angleEncoder = armRotLeader.getEncoder();

        angleEncoder.setPositionConversionFactor(
                (1 / Constants.Arm.GEAR_RATIO) // Rotations of motor shaft devided by
                        // reduction = rotations of mechanism
                        * (2 * Math.PI)); // Rotations * 2pi = rotation in radians

        angleEncoder.setVelocityConversionFactor(
                (1 / Constants.Arm.GEAR_RATIO) * (1.0 / 60.0) * (2 * Math.PI));

        anglePid = armRotLeader.getPIDController();

        setPid(Arm.PidConstants.P, Arm.PidConstants.I, Arm.PidConstants.D);

        armRotLeader.setSmartCurrentLimit(60);
        armRotFollower.setSmartCurrentLimit(60);

        absRotationEncoder = new DutyCycleEncoder(Arm.ABS_ENCODER);

        /* Limit Switches */
        armTopLimitSwitch = new DigitalInput(Arm.TOP_LIMIT_SWITCH);
        armBottomLimitSwitch = new DigitalInput(Arm.BOTTOM_LIMIT_SWITCH);

        armRotLeader.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true);
        armRotLeader.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true);


        armRotLeader.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, (float) Arm.MAX_ROT);
        armRotLeader.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, (float) Arm.MIN_ROT);

        seedArmPosition(true);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.rotation = angleEncoder.getPosition();
        inputs.armRotationSpeed = angleEncoder.getVelocity();
        inputs.absArmRot = getAbsolutePosition();
        inputs.absArmEncoder =
                Angles.wrapAnglePi(-absRotationEncoder.getAbsolutePosition() * 2 * Math.PI);
        inputs.absArmEncoderDeg = Math.toDegrees(inputs.absArmEncoder);

        inputs.topRotationLimit = armTopLimitSwitch.get();
        inputs.bottomRotationLimit = armBottomLimitSwitch.get();

        inputs.encoderConnected = absRotationEncoder.isConnected();
        inputs.volts = armRotLeader.getBusVoltage() * armRotLeader.getAppliedOutput();

        inputs.rotationDeg = Units.radiansToDegrees(inputs.rotation);
    }

    /** Updates the arm position setpoint. */
    @Override
    public void setRotationSetpoint(double rot, double arbFFVolts) {
        anglePid.setReference(
                rot,
                CANSparkMax.ControlType.kPosition,
                0,
                arbFFVolts,
                SparkPIDController.ArbFFUnits.kVoltage);
    }

    @Override
    public void setArmSpeed(double percent) {
        armRotLeader.set(percent);
    }

    @Override
    public void setArmVolts(double volts) {
        System.out.println(volts);
        armRotLeader.setVoltage(volts);
    }

    // TODO: ADD LIMIT SWITCH IMPL
    @Override
    public void checkLimitSwitches() {
        // If velocity 0 return
        // If velocity == Math.abs velocity and top limit switch hit
        // Check arm velocity,

        //        if (armTopLimitSwitch.get() && armRotLeader.getSelectedSensorVelocity() > 0)
        //            armRotLeader.set(ControlMode.PercentOutput, 0);
        //        if (armBottomLimitSwitch.get() && armRotLeader.getSelectedSensorVelocity() < 0)
        //            armRotLeader.set(ControlMode.PercentOutput, 0);

        //    if (extensionBottomLimitSwitch.get() && armExtension.getSelectedSensorVelocity() < 0)
        //        armExtension.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void seedArmPosition(boolean assumeStarting) {
        if (absRotationEncoder.isConnected()) {
            angleEncoder.setPosition(getAbsolutePosition());
        } else if (assumeStarting) {
            System.out.printf(
                    "Arm absolute rotation encoder disconnected, assuming position %s%n",
                    Arm.STARTING_POS);
            angleEncoder.setPosition(Arm.STARTING_POS);
        }
    }

    private double getAbsolutePosition() {
        return Angles.wrapAnglePi(
                -absRotationEncoder.getAbsolutePosition() * 2 * Math.PI - Arm.ABS_ENCODER_OFFSET);
    }

    @Override
    public void setBrake(boolean brake) {
        armRotLeader.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
        armRotFollower.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void setPid(double kP, double kI, double kD) {
        anglePid.setP(kP);
        anglePid.setI(kI);
        anglePid.setD(kD);
    }

    @Override
    public void stop() {
        armRotLeader.stopMotor();
        armRotFollower.stopMotor();
    }
}
