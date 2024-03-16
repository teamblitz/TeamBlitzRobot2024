package frc.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.TalonFX;
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


    public ClimberIOKraken() {

        left = new TalonFX(17);
        right = new TalonFX(18);
    }

    @Override
    public void setSpeedLeft (double speed) {
        left.set(speed);
    }

    @Override
    public void setSpeedRight (double speed) {
        right.set(speed);
    }
        
    @Override   
    public void setSetpointLeft(double extension) {
        left.set(extension);
    }
    

    @Override
    public void setSetpointRight(double extension) {
        right.set(extension);
    }
}


