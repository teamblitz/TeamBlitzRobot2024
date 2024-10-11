package frc.robot.subsystems.drive.control;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.notes.NoteVisionIO;
import java.util.Optional;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.littletonrobotics.junction.Logger;

public class NoteAssistFilter extends ChassisSpeedFilter {

    //    private final Supplier<Translation2d> notePose;
    private final NoteVisionIO.NoteVisionInputs noteVisionInputs;

    public NoteAssistFilter(Drive drive, NoteVisionIO.NoteVisionInputs noteVisionInputs) {
        super(drive, true);
        this.noteVisionInputs = noteVisionInputs;
    }

    @Override
    public ChassisSpeeds apply(ChassisSpeeds initialSpeeds) {
        double timestamp = noteVisionInputs.timestampCapture;

        // Find out where the robot was when we saw the note
        if (Timer.getFPGATimestamp() - timestamp > 1) {
            System.out.println(
                    "NoteAssist: LL Note Position too latent! expected less than 1s latency, got "
                            + (Timer.getFPGATimestamp() - timestamp)
                            + " seconds. Ignoring reading!");
            return initialSpeeds;
        }

        Optional<Pose2d> oldPoseSample =
                drive.samplePreviousPose(noteVisionInputs.timestampCapture);
        if (oldPoseSample.isEmpty()) {
            System.out.println("NoteAssist: No old pose data for timestamp!");
            return initialSpeeds;
        }

        double botSpaceXLatent = noteVisionInputs.botSpaceX;
        double botSpaceYLatent = noteVisionInputs.botSpaceY;


        Pose2d notePoseField =
                oldPoseSample
                        .get()
                        .transformBy(
                                new Transform2d(
                                        botSpaceXLatent, botSpaceYLatent, new Rotation2d()));

        Logger.recordOutput("vision/note/notePoseField", notePoseField);


        // Object's pose in the field coordinate system (Pose2d objectPoseInField)
// Robot's pose in the field coordinate system (Pose2d robotPoseInField)

// Step 1: Compute the relative translation by "subtracting" the robot's position
        Transform2d fieldToRobotSpace = new Transform2d(
                drive.getEstimatedPose().getTranslation().unaryMinus(),
                drive.getEstimatedPose().getRotation().unaryMinus());

// Step 2: Apply this inverse transform to the object's pose
        Pose2d notePoseBotSpace = notePoseField.transformBy(fieldToRobotSpace);

        Logger.recordOutput("vision/note/notePoseBot", notePoseBotSpace);


        Vector2D notePosition = new Vector2D(notePoseBotSpace.getX(), notePoseBotSpace.getY());

        if (notePosition.getNorm() != 0) return initialSpeeds;

        Vector2D noteDirection = notePosition.normalize();

        Vector2D driverVelocity =
                new Vector2D(initialSpeeds.vxMetersPerSecond, initialSpeeds.vyMetersPerSecond);
//        Vector2D driverDirection = driverVelocity.normalize();

        Logger.recordOutput("noteAssist/driver/x", driverVelocity.getX());
        Logger.recordOutput("noteAssist/driver/y", driverVelocity.getY());

        //        double adjustedAngle =
        // Constants.Drive.NoteAssist.ACTIVATION_FUNCTION.applyAsDouble(Vector2D.angle(noteDirection, driverDirection));

        //        Vector2D adjustedDirection = new Vector2D()

        Vector2D adjustedVelocity =
                noteDirection.scalarMultiply(driverVelocity.dotProduct(noteDirection));

        Logger.recordOutput("noteAssist/adjusted/x", adjustedVelocity.getX());
        Logger.recordOutput("noteAssist/adjusted/y", adjustedVelocity.getY());

        return new ChassisSpeeds(
                adjustedVelocity.getX(),
                adjustedVelocity.getY(),
                initialSpeeds.omegaRadiansPerSecond);
    }
}
