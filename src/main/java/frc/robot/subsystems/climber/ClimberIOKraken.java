package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.*;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.math.Angles;
import frc.robot.Constants;
import frc.robot.Constants.Climber;
import com.ctre.phoenix6.configs.TalonFXConfiguration;


public class ClimberIOKraken implements ClimberIO {

    public final TalonFX left;
    public final TalonFX right;

    public final PositionVoltage closedLoopPosition = new PositionVoltage(0);
    public final MotionMagic

    public ClimberIOKraken() {
        left = new TalonFX(Climber.LEFT_MOTOR_ID); //17
        right = new TalonFX(Climber.RIGHT_MOTOR_ID); //18

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake);

        config.CurrentLimits
                .withStatorCurrentLimit(Climber.CURRENT_LIMIT);

        config.Feedback.withSensorToMechanismRatio(
                Climber.GEAR_RATIO * (Climber.SPOOL_DIAMETER * Math.PI)
        );

        left.getConfigurator().apply(config.MotorOutput.withInverted(Climber.LEFT_INVERT));
        right.getConfigurator().apply(config.MotorOutput.withInverted(Climber.RIGHT_INVERT));

        left.optimizeBusUtilization();
        right.optimizeBusUtilization();

//        BaseStatusSignal.setUpdateFrequencyForAll(100, left.getVelocity());
        left.configVoltageComSaturation(12);
        left.enableVoltageCompensation(true);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.positionLeft = left.getPosition().getValueAsDouble();
        inputs.positionRight = right.getPosition().getValueAsDouble();

        inputs.velocityLeft = left.getVelocity().getValueAsDouble();
        inputs.velocityRight = right.getVelocity().getValueAsDouble();

        inputs.voltsLeft = left.getMotorVoltage().getValueAsDouble();
        inputs.voltsRight = right.getMotorVoltage().getValueAsDouble();

        inputs.torqueCurrentLeft = left.getTorqueCurrent().getValueAsDouble();
        inputs.torqueCurrentRight = right.getTorqueCurrent().getValueAsDouble();
    }

    @Override
    public void setSpeedLeft(double speed) {
        left.set(speed);
    }

    @Override
    public void setSpeedRight(double speed) {
        right.set(speed);
    }
        
    @Override   
    public void setSetpointLeft(double extension) {
        left.setControl(closedLoopPosition.withPosition(extension).with);
    }
    

    @Override
    public void setSetpointRight(double extension) {
        // right.set(extension);
    }
}



