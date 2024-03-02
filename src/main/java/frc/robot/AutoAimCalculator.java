package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.lib.math.Trajectories;

import java.util.Vector;

public class AutoAimCalculator {
    private AutoAimCalculator() {}

    /**
     * Puts the bot and the goal into bot goal space with the bot as the origin and the goal as a point along a vertical plane
     */
    public static Pose2d calculateBotToGoal2d(Pose3d botPose, Pose3d goalPose) {
        double horizontalDist = Math.hypot(botPose.getX() - goalPose.getX(), botPose.getY()) - goalPose.getY();
        double vertDist = goalPose.getY() - botPose.getY();

        return new Pose2d(horizontalDist, vertDist, Rotation2d.fromRadians(0));
    }

    public static Transform2d calculateCenterRotationToShooter(Transform2d centerRotationToShooterAtZero, Rotation2d rotation) {
        return new Transform2d(centerRotationToShooterAtZero.getTranslation().rotateBy(rotation), Rotation2d.fromRadians(0));
    }

    public static double calculateArmAngle(Pose3d botPose, Pose3d goalPose, Transform2d botToCenterRot, Transform2d centerRotationToShooterAtZero, double shooterAngleOffset, double initialVelocity) {
        Pose2d goalPose2d = calculateBotToGoal2d(botPose, goalPose);


        double armRot = Units.degreesToRadians(45); // Start at 45 degrees


        for (int i = 0; i < 3; i++) {
            Pose2d shooterPose = new Pose2d()
                    .transformBy(botToCenterRot)
                    .transformBy(calculateCenterRotationToShooter(centerRotationToShooterAtZero, Rotation2d.fromRadians(armRot)));

            Translation2d shooterToGoal = new Transform2d(goalPose2d, shooterPose).getTranslation();

            double shooterAngle = Trajectories.angleRequiredToHitCoordinate(shooterToGoal.getX(), shooterToGoal.getY(), initialVelocity, -9.81);

            armRot = Math.PI - (shooterAngle + shooterAngleOffset);
        }
        return armRot;
    }
}
