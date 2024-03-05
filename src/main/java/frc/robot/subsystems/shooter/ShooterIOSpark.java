package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class ShooterIOSpark implements ShooterIO {

    private final CANSparkMax top;
    private final CANSparkMax bottom;

    private final SparkPIDController pidTop;
    private final SparkPIDController pidBottom;

    private final SimpleMotorFeedforward feedforwardTop;
    private final SimpleMotorFeedforward feedforwardBottom;

    public ShooterIOSpark() {
        top = new CANSparkMax(Constants.Shooter.Spark.SPARK_TOP, MotorType.kBrushless);
        bottom = new CANSparkMax(Constants.Shooter.Spark.SPARK_BOTTOM, MotorType.kBrushless);

        top.restoreFactoryDefaults();
        top.setSmartCurrentLimit(Constants.Shooter.CURRENT_LIMIT);
        top.setOpenLoopRampRate(0.5);
        top.setClosedLoopRampRate(.2);

        bottom.restoreFactoryDefaults();
        bottom.setSmartCurrentLimit(Constants.Shooter.CURRENT_LIMIT);
        bottom.setOpenLoopRampRate(0.5);
        bottom.setClosedLoopRampRate(.2);


        pidTop = top.getPIDController();
        pidBottom = bottom.getPIDController();

        pidTop.setP(Constants.Shooter.Spark.PID_TOP_P);
        pidTop.setI(Constants.Shooter.Spark.PID_TOP_I);
        pidTop.setD(Constants.Shooter.Spark.PID_TOP_D);

        pidBottom.setP(Constants.Shooter.Spark.PID_BOTTOM_P);
        pidBottom.setI(Constants.Shooter.Spark.PID_BOTTOM_I);
        pidBottom.setD(Constants.Shooter.Spark.PID_BOTTOM_D);

        feedforwardTop = new SimpleMotorFeedforward(
                Constants.Shooter.Spark.FF_TOP_KS,
                Constants.Shooter.Spark.FF_TOP_KV,
                Constants.Shooter.Spark.FF_TOP_KA);

        SmartDashboard.putNumber("MAX SHOOT", feedforwardTop.maxAchievableVelocity(12, 30));

        feedforwardBottom = new SimpleMotorFeedforward(
                Constants.Shooter.Spark.FF_BOTTOM_KS,
                Constants.Shooter.Spark.FF_BOTTOM_KV,
                Constants.Shooter.Spark.FF_BOTTOM_KA);

        top.getEncoder().setVelocityConversionFactor(
                (1.0 / 60.0) * (Math.PI * 2 * Units.inchesToMeters(2))
        );
        bottom.getEncoder().setVelocityConversionFactor(
                (1.0 / 60.0) * (Math.PI * 2 * Units.inchesToMeters(2))
        );
    }

    @Override
    public void setPercent(double speed) {
        top.set(speed);
        bottom.set(speed);
    }

    @Override
    public void setVolts(double volts) {
        top.setVoltage(volts);
        bottom.setVoltage(volts);
    }

    @Override
    public void setSetpoint(double velocity) {
        pidTop.setReference(
                velocity,
                CANSparkBase.ControlType.kVelocity,
                0,
                feedforwardTop.calculate(velocity),
                SparkPIDController.ArbFFUnits.kVoltage);
        pidBottom.setReference(
                velocity,
                CANSparkBase.ControlType.kVelocity,
                0,
                feedforwardBottom.calculate(velocity),
                SparkPIDController.ArbFFUnits.kVoltage);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.rpmTop = top.getEncoder().getVelocity();
        inputs.rpmBottom = bottom.getEncoder().getVelocity();

        inputs.currentTop = top.getOutputCurrent();
        inputs.currentBottom = top.getOutputCurrent();
    }
}
