/* Big thanks to Team 364 for the base code. */

package frc.robot.subsystems.drive.swerveModule;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.util.ModuleStateOptimizer;
import frc.robot.Constants;
import frc.robot.subsystems.drive.SwerveModuleIO;
import frc.robot.subsystems.drive.SwerveModuleInputsAutoLogged;

import frc.robot.subsystems.drive.swerveModule.angle.AngleMotorIO;
import frc.robot.subsystems.drive.swerveModule.angle.AngleMotorInputsAutoLogged;
import frc.robot.subsystems.drive.swerveModule.drive.DriveMotorIO;
import frc.robot.subsystems.drive.swerveModule.drive.DriveMotorInputsAutoLogged;
import frc.robot.subsystems.drive.swerveModule.encoder.EncoderIO;
import frc.robot.subsystems.drive.swerveModule.encoder.EncoderIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
    public final int moduleNumber;
    private final SwerveModuleInputsAutoLogged inputs = new SwerveModuleInputsAutoLogged();
    private Rotation2d lastAngle;

    private final AngleMotorIO angleMotor;
    private final DriveMotorIO driveMotor;
    private final EncoderIO absoluteEncoder;

    private final AngleMotorInputsAutoLogged angleMotorInputs = new AngleMotorInputsAutoLogged();
    private final DriveMotorInputsAutoLogged driveMotorInputs = new DriveMotorInputsAutoLogged();
    private final EncoderIOInputsAutoLogged encoderInputs = new EncoderIOInputsAutoLogged();

    private final SimpleMotorFeedforward driveFeedforward =
            new SimpleMotorFeedforward(
                    Constants.Swerve.DRIVE_KS,
                    Constants.Swerve.DRIVE_KV,
                    Constants.Swerve.DRIVE_KA);
    private final String logKey;


    public SwerveModule(int moduleNumber, AngleMotorIO angleMotor, DriveMotorIO driveMotor, EncoderIO absoluteEncoder) {
        this.moduleNumber = moduleNumber;
        this.angleMotor = angleMotor;
        this.driveMotor = driveMotor;
        this.absoluteEncoder = absoluteEncoder;

        // hehe
        absoluteEncoder.updateInputs(encoderInputs);
        angleMotor.seedPosition(encoderInputs.position);

        lastAngle = getAngle();

        logKey = "swerve/mod" + moduleNumber;
    }

    public void configAnglePid(double p, double i, double d) {
        angleMotor.configurePID(p, i, d);
    }

    public void configDrivePid(double p, double i, double d) {
        driveMotor.configurePID(p, i, d);
    }

    public void periodic() {
        angleMotor.updateInputs(angleMotorInputs);
        driveMotor.updateInputs(driveMotorInputs);
        absoluteEncoder.updateInputs(encoderInputs);

        Logger.processInputs(logKey + "/angle", angleMotorInputs);
        Logger.processInputs(logKey + "/drive", driveMotorInputs);
        Logger.processInputs(logKey + "/absEncoder", inputs);
    }

    public void setDesiredState(
            SwerveModuleState desiredState, boolean isOpenLoop, boolean tuning, boolean parking) {

        desiredState = ModuleStateOptimizer.optimize(desiredState, getState().angle);

        setAngle(desiredState, tuning, parking);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.MAX_SPEED;
            driveMotor.setDrivePercent(percentOutput);
            Logger.recordOutput(logKey + "/drivePercent", percentOutput);
        } else {
            Logger.recordOutput(logKey + "/speedSetpoint", desiredState.speedMetersPerSecond);
            driveMotor.setSetpoint(
                    desiredState.speedMetersPerSecond,
                    driveFeedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState, boolean tuning, boolean parking) {
        Rotation2d angle =
                (!(tuning || parking)
                                && Math.abs(desiredState.speedMetersPerSecond)
                                        <= (Constants.Swerve.MAX_SPEED * 0.01))
                        ? lastAngle
                        : desiredState.angle; // Prevent rotating module if speed is less than 1%.
        angleMotor.setSetpoint(angle.getDegrees());
        Logger.recordOutput(logKey + "/AngleSetpoint", angle.getDegrees());
        lastAngle = angle;
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(inputs.anglePositionDegrees);
    }

    public Rotation2d getAbsoluteAngle() {
        return Rotation2d.fromDegrees(inputs.absoluteEncoderPositionDegrees);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(inputs.speedMetersPerSecond, getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(inputs.drivePositionMeters, getAngle());
    }

    public void setBrakeMode(boolean enabled) {
        driveMotor.setBrakeMode(enabled);
    }
}
