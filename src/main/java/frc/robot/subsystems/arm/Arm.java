package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.BlitzSubsystem;
import frc.lib.MutableReference;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

/**
 * Maybe divide this into 2 subsystems, depends on how we want to control it. The current way we do
 * this, 2 subsystems is ideal (and is kinda what we are pseudo doing)
 */
public class Arm extends SubsystemBase implements BlitzSubsystem {

    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    public Arm(ArmIO io) {
        this.io = io;

        // Do this, but smarter
        Commands.waitSeconds(8)
                .andThen(io::seedArmPosition)
                .andThen(() -> io.setArmRotationSpeed(0)) // Set the arm to 0 to end on board pid
                // loop
                .ignoringDisable(true)
                .schedule();

        // Constructor of armio?
        io.seedArmPosition();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("arm", inputs);
    }

    public void updateRotation(double degrees, double velocity) {
        Logger.recordOutput("arm/wanted_rotation", degrees);
        io.setRotationSetpoint(
                degrees,
                0); // Don't do feed forward as we have no way to model this with the spring-loaded
        // arm
    }

    public double getRotation() {
        return inputs.armRot;
    }

    public double getRotationSpeed() {
        return inputs.armRotationSpeed;
    }

    public void setArmRotationSpeed(double percent) {
        io.setArmRotationSpeed(percent);
    }

    public Command rotateToCommandOld(double degrees) {
        double goal = MathUtil.clamp(degrees, Constants.Arm.MIN_ROT, Constants.Arm.MAX_ROT);

        MutableReference<TrapezoidProfile> profile = new MutableReference<>();
        Timer timer = new Timer();

        return runOnce(
                        () -> {
                            profile.set(
                                    new TrapezoidProfile(
                                            new TrapezoidProfile.Constraints(
                                                    Constants.Arm.ROTATION_VELOCITY,
                                                    Constants.Arm.ROTATION_ACCELERATION),
                                            new TrapezoidProfile.State(goal, 0),
                                            new TrapezoidProfile.State(
                                                    getRotation(), getRotationSpeed())));
                            timer.restart();
                        })
                .andThen(
                        run(
                                () -> {
                                    TrapezoidProfile.State setpoint =
                                            profile.get().calculate(timer.get());
                                    updateRotation(setpoint.position, setpoint.velocity);
                                }))
                .until(() -> profile.get().isFinished(timer.get()))
                .finallyDo(
                        (interrupted) ->
                                updateRotation(profile.get().calculate(timer.get()).position, 0));
    }

//    public Command rotateToCommand(double goal) {
//        TrapezoidProfile profile = new TrapezoidProfile(
//                new TrapezoidProfile.Constraints(
//                        Constants.Arm.ROTATION_VELOCITY,
//                        Constants.Arm.ROTATION_ACCELERATION));
//
//        MutableReference<TrapezoidProfile.State> lastState = new MutableReference<>();
//
//        runOnce(
//                () -> lastState.set(new TrapezoidProfile.State(
//                        inputs.armRot,
//                        inputs.armRotationSpeed
//                ))
//        ).andThen(
//                run(
//                        () -> {
//                            TrapezoidProfile.State setpoint =
//                                    profile.calculate(Robot.g);
//                            updateRotation(setpoint.position, setpoint.velocity);
//                        }
//                )
//        )
//        return null;
//    }
}
