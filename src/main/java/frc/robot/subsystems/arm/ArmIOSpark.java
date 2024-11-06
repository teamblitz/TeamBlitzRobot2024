package frc.robot.subsystems.arm;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import frc.lib.math.Angles;
import frc.robot.Constants;
import frc.robot.Constants.Arm;

public class ArmIOSpark implements ArmIO {

    private final SparkMax armRotLeader;
    private final SparkMax armRotFollower;
    private final RelativeEncoder angleEncoder;
    private final SparkClosedLoopController anglePid;

    private final DutyCycleEncoder absRotationEncoder;
    private final Encoder quadEncoder;

    boolean useInternalEncoder;
    double quadOffset = 0;
    private PIDController pid;

    //    private final DigitalInput armTopLimitSwitch;
    //    private final DigitalInput armBottomLimitSwitch;

    public ArmIOSpark(boolean useInternalEncoder) {
        /* Arm Rotation */
        armRotLeader = new SparkMax(Arm.ARM_ROT_LEADER, MotorType.kBrushless);
        armRotFollower =
                new SparkMax(Arm.ARM_ROT_FOLLOWER, MotorType.kBrushless);

        this.useInternalEncoder = useInternalEncoder;


        SparkMaxConfig configLeader = new SparkMaxConfig();
        SparkMaxConfig configFollower = new SparkMaxConfig();

        configLeader.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
        configFollower.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);

        configLeader.openLoopRampRate(Arm.OPEN_LOOP_RAMP);
        configFollower.openLoopRampRate(Arm.OPEN_LOOP_RAMP);

        configLeader.smartCurrentLimit(Arm.CURRENT_LIMIT);
        configFollower.smartCurrentLimit(Arm.CURRENT_LIMIT);

        configLeader.inverted(false);
        configFollower.follow(armRotLeader, true);

        armRotLeader.configure(configLeader, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        armRotFollower.configure(configFollower, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        
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
        quadEncoder = new Encoder(Arm.QUAD_A, Arm.QUAD_B, true);

        quadEncoder.setDistancePerPulse(1 / (2048 * 2 * Math.PI));

        /* Limit Switches */
        //        armTopLimitSwitch = new DigitalInput(Arm.TOP_LIMIT_SWITCH);
        //        armBottomLimitSwitch = new DigitalInput(Arm.BOTTOM_LIMIT_SWITCH);

        armRotLeader.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true);
        armRotLeader.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true);

        armRotLeader.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, (float) Arm.MAX_ROT);
        armRotLeader.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, (float) Arm.MIN_ROT);

        seedArmPosition(true);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.rotation = getPosition();
        inputs.armRotationSpeed = angleEncoder.getVelocity();
        inputs.absArmRot = getAbsolutePosition();
        inputs.absArmEncoder =
                Angles.wrapAnglePi(-absRotationEncoder.getAbsolutePosition() * 2 * Math.PI);
        inputs.absArmEncoderDeg = Math.toDegrees(inputs.absArmEncoder);

        //        inputs.topRotationLimit = armTopLimitSwitch.get();
        //        inputs.bottomRotationLimit = armBottomLimitSwitch.get();

        inputs.encoderConnected = absRotationEncoder.isConnected();
        inputs.volts = armRotLeader.getBusVoltage() * armRotLeader.getAppliedOutput();

        inputs.rotationDeg = Units.radiansToDegrees(inputs.rotation);
    }

    /** Updates the arm position setpoint. */
    @Override
    public void setRotationSetpoint(double rot, double arbFFVolts) {
        if (useInternalEncoder) {
            anglePid.setReference(
                    rot,
                    CANSparkMax.ControlType.kPosition,
                    0,
                    arbFFVolts,
                    SparkPIDController.ArbFFUnits.kVoltage);
        } else {
            armRotLeader.setVoltage(pid.calculate(getPosition(), rot) + arbFFVolts);
        }
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

    @Override
    public void seedArmPosition(boolean assumeStarting) {
        if (absRotationEncoder.isConnected()) {
            angleEncoder.setPosition(getAbsolutePosition());
            quadOffset = getAbsolutePosition() - quadEncoder.getDistance();
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

    private double getPosition() {
        return useInternalEncoder
                ? Angles.wrapAnglePi(quadEncoder.getDistance() + quadOffset)
                : angleEncoder.getPosition();
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

        pid = new PIDController(kP, kI, kD);
    }

    @Override
    public void stop() {
        armRotLeader.stopMotor();
        armRotFollower.stopMotor();
    }
}
