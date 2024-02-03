package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;

public class ShooterIOSpark implements ShooterIO {

    private final CANSparkMax top;
    private final CANSparkMax bottom;

    private final SparkPIDController pidTop;
    private final SparkPIDController pidBottom;

    private final SimpleMotorFeedforward feedforwardTop;
    private final SimpleMotorFeedforward feedforwardBottom;

    public ShooterIOSpark() {
        top = new CANSparkMax(Constants.Shooter.TOP, MotorType.kBrushless);
        bottom = new CANSparkMax(Constants.Shooter.BOTTOM, MotorType.kBrushless);

        top.restoreFactoryDefaults();
        top.setSmartCurrentLimit(Constants.Shooter.CURRENT_LIMIT);
        top.setOpenLoopRampRate(0.5);

        bottom.restoreFactoryDefaults();
        bottom.setSmartCurrentLimit(Constants.Shooter.CURRENT_LIMIT);
        bottom.setOpenLoopRampRate(0.5);

        pidTop = top.getPIDController();
        pidBottom = top.getPIDController();

        pidTop.setP(0);
        pidTop.setI(0);
        pidTop.setD(0);

        pidBottom.setP(0);
        pidBottom.setI(0);
        pidBottom.setD(0);

        feedforwardTop = new SimpleMotorFeedforward(0, 0, 0);
        feedforwardBottom = new SimpleMotorFeedforward(0, 0, 0);
    }

    @Override
    public void setPercent(double speed) {
        top.set(speed);
        bottom.set(speed);
    }

    @Override
    public void setSetpoint(double velocity) {
        pidTop.setReference(
                velocity,
                CANSparkBase.ControlType.kVelocity,
                0,
                feedforwardTop.calculate(velocity),
                SparkPIDController.ArbFFUnits.kVoltage
        );
        pidBottom.setReference(
                velocity,
                CANSparkBase.ControlType.kVelocity,
                0,
                feedforwardBottom.calculate(velocity),
                SparkPIDController.ArbFFUnits.kVoltage
        );
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.rpmTop = top.getEncoder().getVelocity();
        inputs.rpmBottom = bottom.getEncoder().getVelocity();

        inputs.currentTop = top.getOutputCurrent();
        inputs.currentBottom = top.getOutputCurrent();
    }
}
