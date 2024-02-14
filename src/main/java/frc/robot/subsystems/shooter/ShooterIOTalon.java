package frc.robot.subsystems.shooter;

// NOT TALONS
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.Constants;

public class ShooterIOTalon implements ShooterIO {
    private final WPI_VictorSPX top;
    private final WPI_VictorSPX bottom;

    public ShooterIOTalon() {
        top = new WPI_VictorSPX(Constants.Shooter.Talon.TALON_TOP); // CHECK CONSTANTS FOR THIS ID
        bottom =
                new WPI_VictorSPX(
                        Constants.Shooter.Talon.TALON_BOTTOM); // CHECK CONSTANTS FOR THIS ID
    }

    @Override
    public void setPercent(double speed) {
        top.set(speed);
        bottom.set(speed);
    }
}
