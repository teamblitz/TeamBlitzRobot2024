package frc.robot.subsystems.drive.filter;
//
//import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.kinematics.ChassisSpeeds;
//import frc.lib.util.LoggedTunableNumber;
//
//import java.util.function.Supplier;
//
public class AmpAssistFilter {
//    private final Supplier<Double> range;
//    private final Supplier<Double> setpoint;
//
//    private final LoggedTunableNumber p = new LoggedTunableNumber("drive/amp-assist/kP", 1);
////    private final LoggedTunableNumber  = new LoggedTunableNumber("drive/amp-assist/kP");
//
//    public AmpAssistFilter(Supplier<Double> range, Supplier<Double> setpoint) {
//        this.range = range;
//        this.setpoint = setpoint;
//    }
//
//    @Override
//    public boolean fieldRelative() {
//        return true;
//    }
//
//    /**
//     * Applies this function to the given argument.
//     *
//     * @param chassisSpeeds the function argument
//     * @return the function result
//     */
//    @Override
//    public ChassisSpeeds apply(ChassisSpeeds initialSpeeds) {
//        return new ChassisSpeeds(
//                initialSpeeds.vxMetersPerSecond,
//                (setpoint.get() - range.get()) * p.get(),
//                initialSpeeds.omegaRadiansPerSecond
//        );
//    }
}
