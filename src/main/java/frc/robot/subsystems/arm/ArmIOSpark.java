package frc.robot.subsystems.arm;

import com.revrobotics.*;
import com.revrobotics.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
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
        armRotFollower = new SparkMax(Arm.ARM_ROT_FOLLOWER, MotorType.kBrushless);

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

        configLeader.softLimit.forwardSoftLimitEnabled(true);
        configLeader.softLimit.reverseSoftLimitEnabled(true);

        configLeader.softLimit.forwardSoftLimit((float) Arm.MAX_ROT);
        configLeader.softLimit.reverseSoftLimit((float) Arm.MIN_ROT);

        //        configLeader.encoder.velocityConversionFactor()

        armRotLeader.configure(
                configLeader, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        armRotFollower.configure(
                configFollower, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        angleEncoder = armRotLeader.getEncoder();

        angleEncoder.setPositionConversionFactor(
                (1 / Constants.Arm.GEAR_RATIO) // Rotations of motor shaft devided by
                        // reduction = rotations of mechanism
                        * (2 * Math.PI)); // Rotations * 2pi = rotation in radians

        angleEncoder.setVelocityConversionFactor(
                (1 / Constants.Arm.GEAR_RATIO) * (1.0 / 60.0) * (2 * Math.PI));

        anglePid = armRotLeader.getClosedLoopController();

        setPid(Arm.PidConstants.P, Arm.PidConstants.I, Arm.PidConstants.D);

        absRotationEncoder = new DutyCycleEncoder(Arm.ABS_ENCODER);
        quadEncoder = new Encoder(Arm.QUAD_A, Arm.QUAD_B, true);

        quadEncoder.setDistancePerPulse(1 / (2048 * 2 * Math.PI));

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
                    SparkMax.ControlType.kPosition,
                    0,
                    arbFFVolts,
                    SparkClosedLoopController.ArbFFUnits.kVoltage);
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
        armRotLeader.armRotLeader.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
        armRotFollower.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void setPid(double kP, double kI, double kD) {

        SparkMaxConfig wastedMemory = new SparkMaxConfig();

        // TODO: REV LIB 2025: IM QUIRKY AND NEED TO DO A CONFIGURE ROUTINE TO SET PID CONSTANTS.
        wastedMemory.closedLoop.pid(kP, kI, kD);
        armRotLeader.configure(
                wastedMemory, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        pid = new PIDController(kP, kI, kD);
    }

    @Override
    public void stop() {
        armRotLeader.stopMotor();
        armRotFollower.stopMotor();
    }
}
