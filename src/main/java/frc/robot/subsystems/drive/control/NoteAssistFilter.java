package frc.robot.subsystems.drive.control;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import java.util.function.Supplier;

public class NoteAssistFilter extends ChassisSpeedFilter {

    private final Supplier<Translation2d> notePose;

    public NoteAssistFilter(Drive drive, Supplier<Translation2d> notePose) {
        super(drive, true);
        this.notePose = notePose;
    }

    @Override
    public ChassisSpeeds apply(ChassisSpeeds initialSpeeds) {
        Translation2d pose = notePose.get();
        Vector2D notePosition = new Vector2D(pose.getX(),pose.getY());
        Vector2D noteDirection = notePosition.normalize();

        Vector2D driverVelocity = new Vector2D(initialSpeeds.vxMetersPerSecond, initialSpeeds.vyMetersPerSecond);
        Vector2D driverDirection = driverVelocity.normalize();

//        double adjustedAngle = Constants.Drive.NoteAssist.ACTIVATION_FUNCTION.applyAsDouble(Vector2D.angle(noteDirection, driverDirection));

//        Vector2D adjustedDirection = new Vector2D()

        Vector2D adjustedVelocity = noteDirection.scalarMultiply(driverVelocity.dotProduct(noteDirection));

        return new ChassisSpeeds(
                adjustedVelocity.getX(),
                adjustedVelocity.getY(),
                initialSpeeds.omegaRadiansPerSecond);
    }
}
