package frc.robot.subsystems.arm;


import com.revrobotics.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.math.Angles;
import frc.lib.math.Conversions;
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
        armRotFollower = new CANSparkMax(Arm.ARM_ROT_FOLLOWER, CANSparkLowLevel.MotorType.kBrushless);

        armRotLeader.restoreFactoryDefaults();
        armRotFollower.restoreFactoryDefaults();

        armRotLeader.setIdleMode(CANSparkBase.IdleMode.kBrake);
        armRotFollower.setIdleMode(CANSparkBase.IdleMode.kBrake);

        armRotLeader.setOpenLoopRampRate(Arm.RAMP_RATE);
        armRotFollower.setOpenLoopRampRate(Arm.RAMP_RATE);

        armRotLeader.setInverted(false);
        armRotFollower.follow(armRotLeader, true);

        angleEncoder = armRotLeader.getEncoder();

        //TODO Fix this ratio
        armRotLeader.getEncoder().setPositionConversionFactor(
                (1 / Constants.Swerve.ANGLE_GEAR_RATIO) // We do 1 over the gear ratio because 1
                        // rotation of the motor is < 1 rotation of
                        // the module
                        * 360); // 1/360 rotations is 1 degree, 1 rotation is 360 degrees.


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
        inputs.armRot =
                Conversions.falconToDegrees(
                        armRotLeader.getSelectedSensorPosition(), Arm.ROTATION_GEAR_RATIO);
        inputs.armRotationSpeed =
                Conversions.falconToDegrees(
                                armRotLeader.getSelectedSensorVelocity(), Arm.ROTATION_GEAR_RATIO)
                        / 10.0;
        inputs.absArmRot = getAbsolutePosition();
        inputs.absArmEncoder = Angles.wrapAngle180(-absRotationEncoder.getAbsolutePosition() * 360);

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
        armRotLeader.set(ControlMode.PercentOutput, percent);
    }


    @Override
    public void checkLimitSwitches() {
        // If velocity 0 return
        // If velocity == Math.abs velocity and top limit switch hit
        // Check arm velocity,

        //        if (armTopLimitSwitch.get() && armRotLeader.getSelectedSensorVelocity() > 0)
        //            armRotLeader.set(ControlMode.PercentOutput, 0);
        //        if (armBottomLimitSwitch.get() && armRotLeader.getSelectedSensorVelocity() < 0)
        //            armRotLeader.set(ControlMode.PercentOutput, 0);

//        if (extensionBottomLimitSwitch.get() && armExtension.getSelectedSensorVelocity() < 0)
//            armExtension.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void seedArmPosition() {
        if (absRotationEncoder.isConnected()) {
            armRotLeader.setSelectedSensorPosition(
                    Conversions.degreesToFalcon(getAbsolutePosition(), Arm.ROTATION_GEAR_RATIO));
        } else {
            System.out.printf(
                    "Arm absolute rotation encoder disconnected, assuming position %s%n",
                    Arm.STARTING_ROTATION);
            armRotLeader.setSelectedSensorPosition(
                    Conversions.degreesToFalcon(Arm.STARTING_ROTATION, Arm.ROTATION_GEAR_RATIO));
        }
    }

    private double getAbsolutePosition() {
        return Angles.wrapAngle180(
                Angles.wrapAngle180(
                        -absRotationEncoder.getAbsolutePosition() * 360 - Arm.ARM_ROT_OFFSET));
    }
}
