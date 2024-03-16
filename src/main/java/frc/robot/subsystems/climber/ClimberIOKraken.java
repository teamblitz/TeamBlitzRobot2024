package frc.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.*;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.math.Angles;
import frc.robot.Constants;
import frc.robot.Constants.Arm;

public class ClimberIOKraken implements ClimberIO {

    public final TalonFX left;
    public final TalonFX right;

    private final DigitalInput topLimitSwitch;
    private final DigitalInput bottomLimitSwitch;

    public ClimberIOKraken() {

        left = new TalonFX();
        right = new TalonFX();



        armRotLeader.setIdleMode(IdleMode.kBrake);
        armRotFollower.setIdleMode(IdleMode.kBrake);

        armRotLeader.setOpenLoopRampRate(Arm.RAMP_RATE);
        armRotFollower.setOpenLoopRampRate(Arm.RAMP_RATE);

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

        anglePid.setP(Arm.PidConstants.P);
        anglePid.setI(Arm.PidConstants.I);
        anglePid.setD(Arm.PidConstants.D);

        armRotLeader.setSmartCurrentLimit(60);
        armRotFollower.setSmartCurrentLimit(60);

        absRotationEncoder = new DutyCycleEncoder(Arm.ABS_ENCODER);

        /* Limit Switches */
        armTopLimitSwitch = new DigitalInput(Arm.TOP_LIMIT_SWITCH);
        armBottomLimitSwitch = new DigitalInput(Arm.BOTTOM_LIMIT_SWITCH);

    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.rotation = angleEncoder.getPosition();
        inputs.armRotationSpeed = angleEncoder.getVelocity();
        inputs.absArmRot = getAbsolutePosition();
        inputs.absArmEncoder =
                Angles.wrapAnglePi(-absRotationEncoder.getAbsolutePosition() * 2 * Math.PI);

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
    @Override
    public void setBrake(boolean brake) {
        armRotLeader.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
        armRotFollower.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }
}
