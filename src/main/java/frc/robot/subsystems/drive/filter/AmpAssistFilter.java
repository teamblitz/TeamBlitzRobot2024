package frc.robot.subsystems.drive.filter;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive;

public class AmpAssistFilter extends ChassisSpeedsFilter {
    private final TrapezoidProfile motionProfile =
            new TrapezoidProfile(new TrapezoidProfile.Constraints(1, 2));

    private static final LoggedTunableNumber p = new LoggedTunableNumber("drive/amp-assist/kP", 5);

    private TrapezoidProfile.State state = new TrapezoidProfile.State();
    private final TrapezoidProfile.State goal = new TrapezoidProfile.State(.3, 0);

    public AmpAssistFilter(Drive drive) {
        super(drive, true);
    }

    @Override
    public ChassisSpeeds apply(ChassisSpeeds initialSpeeds) {
        state = motionProfile.calculate(Robot.defaultPeriodSecs, state, goal);

        return new ChassisSpeeds(
                initialSpeeds.vxMetersPerSecond,
                (state.position - drive.getRange()) * p.get() + state.velocity,
                initialSpeeds.omegaRadiansPerSecond);
    }

    @Override
    public void reset() {
        state =
                new TrapezoidProfile.State(
                        drive.getRange(), drive.getFieldRelativeSpeeds().vyMetersPerSecond);
    }
}
