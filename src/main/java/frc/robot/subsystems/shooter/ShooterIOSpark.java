package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.Climber;

public class ShooterIOSpark implements ShooterIO {

    private final CANSparkMax top;
    private final CANSparkMax bottom;

    private final SparkPIDController pidTop;
    private final SparkPIDController pidBottom;

    private SimpleMotorFeedforward feedforwardTop;
    private SimpleMotorFeedforward feedforwardBottom;

    public ShooterIOSpark() {
        top = new CANSparkMax(Constants.Shooter.Spark.SPARK_TOP, MotorType.kBrushless);
        bottom = new CANSparkMax(Constants.Shooter.Spark.SPARK_BOTTOM, MotorType.kBrushless);

        top.restoreFactoryDefaults();
        top.setSmartCurrentLimit(Constants.Shooter.CURRENT_LIMIT);
        top.setOpenLoopRampRate(0.5);
        top.setClosedLoopRampRate(.2);
        top.setIdleMode(CANSparkBase.IdleMode.kCoast);

        bottom.restoreFactoryDefaults();
        bottom.setSmartCurrentLimit(Constants.Shooter.CURRENT_LIMIT);
        bottom.setOpenLoopRampRate(0.5);
        bottom.setClosedLoopRampRate(.2);
        bottom.setIdleMode(CANSparkBase.IdleMode.kCoast);


        pidTop = top.getPIDController();
        pidBottom = bottom.getPIDController();

        pidTop.setP(Constants.Shooter.Spark.PID_TOP_P);
        pidTop.setI(Constants.Shooter.Spark.PID_TOP_I);
        pidTop.setD(Constants.Shooter.Spark.PID_TOP_D);

        pidBottom.setP(Constants.Shooter.Spark.PID_BOTTOM_P);
        pidBottom.setI(Constants.Shooter.Spark.PID_BOTTOM_I);
        pidBottom.setD(Constants.Shooter.Spark.PID_BOTTOM_D);

        feedforwardTop =
                new SimpleMotorFeedforward(
                        Constants.Shooter.Spark.FF_TOP_KS,
                        Constants.Shooter.Spark.FF_TOP_KV,
                        Constants.Shooter.Spark.FF_TOP_KA);

        feedforwardBottom =
                new SimpleMotorFeedforward(
                        Constants.Shooter.Spark.FF_BOTTOM_KS,
                        Constants.Shooter.Spark.FF_BOTTOM_KV,
                        Constants.Shooter.Spark.FF_BOTTOM_KA);

        SmartDashboard.putNumber("MAX SHOOT Top", feedforwardTop.maxAchievableVelocity(12, 0));
        SmartDashboard.putNumber("MAX SHOOT Bottom", feedforwardBottom.maxAchievableVelocity(12, 0));

        top.getEncoder()
                .setVelocityConversionFactor(
                        Constants.Shooter.GEAR_RATIO * (1.0 / 60.0) * (Math.PI * 2 * Units.inchesToMeters(2)));
        bottom.getEncoder()
                .setVelocityConversionFactor(
                        Constants.Shooter.GEAR_RATIO * (1.0 / 60.0) * (Math.PI * 2 * Units.inchesToMeters(2)));

        top.getEncoder()
                .setPositionConversionFactor(
                        Constants.Shooter.GEAR_RATIO * (Math.PI * 2 * Units.inchesToMeters(2)));
        bottom.getEncoder()
                .setPositionConversionFactor(
                        Constants.Shooter.GEAR_RATIO * (Math.PI * 2 * Units.inchesToMeters(2)));


//        pidTop.setSmartMotionAccelStrategy(SparkPIDController.AccelStrategy.kTrapezoidal, 0);
//        pidBottom.setSmartMotionAccelStrategy(SparkPIDController.AccelStrategy.kTrapezoidal, 0);
//
//        pidTop.setSmartMotionMaxAccel(40, 0);
//        pidBottom.setSmartMotionMaxAccel(40, 0);
//
//        pidTop.setSmartMotionMaxAccel(40, 0);
//        pidBottom.setSmartMotionMaxAccel(40, 0);
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
        inputs.currentBottom = bottom.getOutputCurrent();
    }

    @Override
    public void setTopPid(double kP, double kI, double kD) {
        pidTop.setP(kP);
        pidTop.setI(kI);
        pidTop.setD(kD);
    }

    @Override
    public void setBottomPid(double kP, double kI, double kD) {
        pidBottom.setP(kP);
        pidBottom.setI(kI);
        pidBottom.setD(kD);
    }

    @Override
    public void setTopFF(double kS, double kV, double kA) {
        feedforwardTop = new SimpleMotorFeedforward(kS, kV, kA);
    }

    @Override
    public void setBottomFF(double kS, double kV, double kA) {
        feedforwardBottom = new SimpleMotorFeedforward(kS, kV, kA);
    }
}
