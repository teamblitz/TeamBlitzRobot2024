package frc.robot.subsystems.vision.notes;

import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.util.LimelightHelpers;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;

public class NoteVisionIOLimelight implements NoteVisionIO {
    private final String name;
    private final Mat homography =
            Calib3d.findHomography(
                    new MatOfPoint2f( // Camera points
                            new Point(), new Point(), new Point(), new Point()),
                    new MatOfPoint2f( // Real world points
                            new Point(), new Point(), new Point(), new Point()));

    public NoteVisionIOLimelight(String limelightName) {
        this.name = limelightName;
    }

    @Override
    public void updateInputs(NoteVisionInputs inputs) {
        LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(name);
        LimelightHelpers.LimelightTarget_Detector detectorResults = results.targets_Detector[0];

        inputs.tx = detectorResults.tx;
        inputs.ty = detectorResults.ty;
        inputs.valid = results.valid;

        inputs.txPixels = detectorResults.tx_pixels;
        inputs.tyPixels = detectorResults.ty_pixels;

        Mat src = new MatOfPoint2f(new Point(inputs.txPixels, inputs.tyPixels)); // Matrix of vectors to be transformed
        MatOfPoint2f dst = new MatOfPoint2f(); // A mutable matrix for opencv to store the botspace coordinates;

        try {
            Core.perspectiveTransform(src, dst, homography);
            inputs.projectionValid = true;
        } catch (Exception e) {
            System.out.println("OpenCV Homography transformation failed");
            inputs.projectionValid = false;
        }

        Point noteBotSpace = dst.toArray()[0];

        inputs.botSpaceX = noteBotSpace.x;
        inputs.botSpaceY = noteBotSpace.y;

        inputs.timestampCapture = results.timestamp_RIOFPGA_capture; // TODO: Verify this is what we want and need
    }
}
