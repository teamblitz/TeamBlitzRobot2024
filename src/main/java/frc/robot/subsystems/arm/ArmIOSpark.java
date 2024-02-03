package frc.robot.subsystems.arm;

import com.revrobotics.*;
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

        armRotLeader.setIdleMode(CANSparkBase.IdleMode.kBrake);
        armRotFollower.setIdleMode(CANSparkBase.IdleMode.kBrake);

        armRotLeader.setOpenLoopRampRate(Arm.RAMP_RATE);
        armRotFollower.setOpenLoopRampRate(Arm.RAMP_RATE);

        armRotLeader.setInverted(false);
        armRotFollower.follow(armRotLeader, true);

        angleEncoder = armRotLeader.getEncoder();

        armRotLeader
                .getEncoder()
                .setPositionConversionFactor(
                        (1 / Constants.Arm.GEAR_RATIO) // Rotations of motor shaft devided by
                                // reduction = rotations of mechanism
                                * (2 * Math.PI)); // Rotations * 2pi = rotation in radians

        anglePid = armRotLeader.getPIDController();

        anglePid.setP(Arm.PidConstants.P);
        anglePid.setI(Arm.PidConstants.I);
        anglePid.setD(Arm.PidConstants.D);

        armRotLeader.setSmartCurrentLimit(60);
        armRotFollower.setSmartCurrentLimit(60);

        absRotationEncoder = new DutyCycleEncoder(Arm.ABS_ENCODER);

        /* Limit Switches */
        armTopLimitSwitch = new DigitalInput(Arm.TOP_LIMIT_SWITCH);
        armBottomLimitSwitch = new DigitalInput(Arm.BOTTOM_LIMIT_SWITCH);

        seedArmPosition();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.armRot = angleEncoder.getPosition();
        inputs.armRotationSpeed = angleEncoder.getVelocity();
        inputs.absArmRot = getAbsolutePosition();
        inputs.absArmEncoder =
                Angles.wrapAnglePi(-absRotationEncoder.getAbsolutePosition() * 2 * Math.PI);

        inputs.topRotationLimit = armTopLimitSwitch.get();
        inputs.bottomRotationLimit = armBottomLimitSwitch.get();

        inputs.encoderConnected = absRotationEncoder.isConnected();
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
    public void setArmRotationSpeed(double percent) {
        armRotLeader.set(percent);
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
    public void seedArmPosition() {
        if (absRotationEncoder.isConnected()) {
            angleEncoder.setPosition(getAbsolutePosition());
        } else {
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
}
