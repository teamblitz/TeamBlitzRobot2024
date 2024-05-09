package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public final class ManipulatorCommands {
    public static Command intakeGround(Intake intake, Arm arm) {
        return intake.intakeGroundAutomatic()
                .deadlineWith(arm.rotateToCommand(Constants.Arm.Positions.INTAKE, true, true));
    }

    public static Command shootAngle(double angle, Shooter shooter, Arm arm) {
        return arm.rotateToCommand(angle, false).alongWith(shooter.shootCommand());
    }

    private ManipulatorCommands() {}
}
