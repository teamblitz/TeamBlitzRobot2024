package frc.robot.subsystems.shooter;
//NOT TALONS
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;


public class ShooterIOTalon implements ShooterIO {
    private final WPI_VictorSPX top;
    private final WPI_VictorSPX bottom;
    

    public ShooterIOTalon() {
        top = new WPI_VictorSPX(Constants.Shooter.Talon.TALON_TOP); //CHECK CONSTANTS FOR THIS ID
        bottom = new WPI_VictorSPX(Constants.Shooter.Talon.TALON_BOTTOM); //CHECK CONSTANTS FOR THIS ID
    }

     @Override
    public void setPercent(double speed) {
        top.set(speed);
        bottom.set(speed);
    }

    
}

   
