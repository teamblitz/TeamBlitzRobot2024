package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


public class ShooterIOTalon implements ShooterIO {
    private final WPI_TalonSRX top;
    private final WPI_TalonSRX bottom;

     @Override
    public void setPercent(double speed) {
        top.set(speed);
        bottom.set(speed);
    }
}

   
