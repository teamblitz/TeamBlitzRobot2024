package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoAimCalculator;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.Supplier;

public final class ManipulatorCommands {
    public static Command intakeGround(Intake intake, Arm arm) {
        return intake.intakeGroundAutomatic().deadlineWith(arm.setGoal(Arm.Goal.INTAKE));
    }

    public static Command shootSubwoofer(Shooter shooter, Arm arm) {
        return arm.setGoal(Arm.Goal.SUBWOOFER).alongWith(shooter.shootCommand());
    }

    public static Command shootPodium(Shooter shooter, Arm arm) {
        return arm.setGoal(Arm.Goal.PODIUM).alongWith(shooter.shootCommand());
    }

    public static Command shootAim(Shooter shooter, Arm arm, Supplier<Pose2d> poseSupplier) {
        return arm.setGoal(Arm.Goal.AIM)
                .alongWith(
                        shooter.shootClosedLoopCommand(
                                () ->
                                        AutoAimCalculator.calculateShooterSpeedInterpolation(
                                                AutoAimCalculator.calculateDistanceToGoal(
                                                        new Pose3d(poseSupplier.get())))));
    }

    private ManipulatorCommands() {}
}
