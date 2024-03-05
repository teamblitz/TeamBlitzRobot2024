package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.math.Trajectories;
import org.littletonrobotics.junction.Logger;

import java.util.Vector;

public class AutoAimCalculator {
    public static final ShuffleboardTab tab = Shuffleboard.getTab("AutoShoot");

    private AutoAimCalculator() {}

    /**
     * Puts the bot and the goal into bot goal space with the bot as the origin and the goal as a point along a vertical plane
     */
    public static Pose2d calculateBotToGoal2d(Pose3d botPose, Pose3d goalPose) {
        double horizontalDist = Math.hypot(botPose.getX() - goalPose.getX(), botPose.getY() - goalPose.getY());
        Logger.recordOutput("AutoShoot/botPose", botPose);
        Logger.recordOutput("AutoShoot/goalPose", goalPose);



        Logger.recordOutput("AutoShoot/horizontalDist", horizontalDist);

        double vertDist = goalPose.getZ() - botPose.getZ();
        Logger.recordOutput("AutoShoot/vertDist", vertDist);


        return new Pose2d(horizontalDist, vertDist, Rotation2d.fromRadians(0));
    }

    public static Transform2d calculateCenterRotationToShooter(Transform2d centerRotationToShooterAtZero, Rotation2d rotation) {
        return new Transform2d(centerRotationToShooterAtZero.getTranslation().rotateBy(rotation), Rotation2d.fromRadians(0));
    }

    public static double calculateArmAngle(Pose3d botPose, Pose3d goalPose, Transform2d botToCenterRot, Transform2d centerRotationToShooterAtZero, double shooterAngleOffset, double initialVelocity) {
        Pose2d goalPose2d = calculateBotToGoal2d(botPose, goalPose);
        Logger.recordOutput("AutoShoot/goalPose2d", goalPose2d);


        double armRot = Units.degreesToRadians(45); // Start at 45 degrees

        Pose2d shooterPoseDebug = null;
        Translation2d shooterToGoalDebug = null;
        double shooterAngleDebug = 0;

        for (int i = 0; i < 10; i++) {
            Pose2d shooterPose = new Pose2d()
                    .transformBy(botToCenterRot)
                    .transformBy(calculateCenterRotationToShooter(centerRotationToShooterAtZero, Rotation2d.fromRadians(armRot)));

            shooterPoseDebug = shooterPose;

            Translation2d shooterToGoal = new Transform2d(shooterPose, goalPose2d).getTranslation();

            shooterToGoalDebug = shooterToGoal;

            double shooterAngle = Trajectories.angleRequiredToHitCoordinate(shooterToGoal.getX(), shooterToGoal.getY(), initialVelocity, -9.81);
            shooterAngleDebug = shooterAngle;

            armRot = Math.PI/2 - (shooterAngle + shooterAngleOffset);
        }

        Logger.recordOutput("AutoShoot/shooterToGoal", shooterToGoalDebug);
        Logger.recordOutput("AutoShoot/shooterPose", shooterPoseDebug);
        Logger.recordOutput("AutoShoot/shooterAngle", Units.radiansToDegrees(shooterAngleDebug));



        return armRot + Units.degreesToRadians(7);
    }
}
