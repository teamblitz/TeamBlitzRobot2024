/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.COTSSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 *
 * <p>Units: Distance in meters Rotation in radians Time in seconds
 */
public final class Constants {

    public static final Mode simMode = Mode.SIM;

    public static final boolean tuningMode = true;

    public enum Mode {
        /**
         * Running a physics simulator.
         */
        SIM,

        /**
         * Replaying from a log file.
         */
        REPLAY
    }

    public static final class Swerve {
        public static final int PIGEON_ID = 30;
        public static final boolean USE_PIGEON = true;

        public static final COTSSwerveConstants chosenModule =
                COTSSwerveConstants.SDSMK4i(COTSSwerveConstants.driveGearRatios.SDSMK4i_L3);

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.inchesToMeters(24.75);
        public static final double WHEEL_BASE = Units.inchesToMeters(24.75);
        public static final double WHEEL_CIRCUMFERENCE = chosenModule.wheelCircumference;

        /* Motor Inverts */
        public static final boolean ANGLE_MOTOR_INVERT = chosenModule.angleMotorInvert;
        public static final boolean DRIVE_MOTOR_INVERT = chosenModule.driveMotorInvert;

        /* Module Gear Ratios */
        public static final double DRIVE_GEAR_RATIO = chosenModule.driveGearRatio;
        public static final double ANGLE_GEAR_RATIO = chosenModule.angleGearRatio;

        /* Angle Encoder Invert */
        public static final boolean CAN_CODER_INVERT = chosenModule.canCoderInvert;

        public static final int FL = 0; // Front Left Module Index
        public static final int FR = 1; // Front Right Module Index
        public static final int BL = 2; // Back Left Module Index
        public static final int BR = 3; // Back Right Module Index

        public static final List<Translation2d> CENTER_TO_MODULE =
                Arrays.asList(
                        new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                        new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                        new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                        new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

        public static final SwerveDriveKinematics KINEMATICS =
                new SwerveDriveKinematics(
                        CENTER_TO_MODULE.get(FL),
                        CENTER_TO_MODULE.get(FR),
                        CENTER_TO_MODULE.get(BL),
                        CENTER_TO_MODULE.get(BR));

        /* Current Limits
         *
         * Current Limits attempt to prevent the motor from burning out under a stall condition, and prevent the breaker from being tripped.
         *
         * The current limits are from the controller to the motor, and aren't necessarily the same as the current coming from the battery.
         *
         * The smart current limit scales the current limit based on the speed of the motor (to make it better for closed loop control iirc),
         * Secondary limit turns off the motor until the current falls below the limit
         *
         * Not even sure if it does anything because the fuses are 40a
         */
        public static final int DRIVE_SMART_CURRENT_LIMIT = 65;
        public static final int DRIVE_SECONDARY_CURRENT_LIMIT = 80;

        public static final int ANGLE_SMART_CURRENT_LIMIT = 25;
        public static final int ANGLE_SECONDARY_CURRENT_LIMIT = 40;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double OPEN_LOOP_RAMP = 0.15;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        /* Angle Motor PID Values */
        public static final double ANGLE_KP = 0.003;
        public static final double ANGLE_KI = 0.0;
        public static final double ANGLE_KD = 0.0;
        public static final double ANGLE_KF = 0.0; // For now, should remain zero

        /* Drive Motor PID Values */
        /*
         * Some posible gains for kp
         * 0.0012347 60s denominator, don't convert (possible)
         * 0.074084 1s denominator, don't convert] (high)
         *
         * I for some reason don't trust this one
         * 0.00063245 60s denominator, do convert 6.12 : 1
         */
        public static final double DRIVE_KP = 0.0012347;
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        public static final double DRIVE_KF = 0.0; // Same here.

        /* Drive Motor Characterization Values in volts*/
        public static final double DRIVE_KS = (0.19983);
        public static final double DRIVE_KV = (2.3923);
        public static final double DRIVE_KA = (0.47987);

        /* Drive Profiling Values */
        /** Meters per Second */
        public static final double MAX_SPEED = 4.6; // TODO: This must be tuned to specific robot

        /**
         * Radians per Second
         *
         * <p>Can likely be figured out using an equation. Or we can just tornado spin it and see
         * what happens.
         *
         * <p>public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
         * MAX_VELOCITY_METERS_PER_SECOND / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
         * DRIVETRAIN_WHEELBASE_METERS / 2.0);
         *
         * <p>Assuming our robot can still go at 4.6 meters per second (which it can't, this value
         * was taken when we had like nothing on our robot, we can go
         * 10.348178456877813130498318828226534894488969421353632604171 radians per second while
         * spinning
         */
        public static final double MAX_ANGULAR_VELOCITY =
                10.0; // TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final IdleMode ANGLE_NEUTRAL_MODE = IdleMode.kCoast;
        public static final IdleMode DRIVE_NEUTRAL_MODE = IdleMode.kCoast;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int DRIVE_MOTOR_ID = 7;
            public static final int ANGLE_MOTOR_ID = 8;
            public static final int CAN_CODER_ID = 4;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(235.28 + 90);
            public static final SwerveModuleConstants CONSTANTS =
                    new SwerveModuleConstants(
                            DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int DRIVE_MOTOR_ID = 5;
            public static final int ANGLE_MOTOR_ID = 6;
            public static final int CAN_CODER_ID = 2;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(19.68 + 90);
            public static final SwerveModuleConstants CONSTANTS =
                    new SwerveModuleConstants(
                            DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int DRIVE_MOTOR_ID = 11;
            public static final int ANGLE_MOTOR_ID = 12;
            public static final int CAN_CODER_ID = 13;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(18.54 + 90);
            public static final SwerveModuleConstants CONSTANTS =
                    new SwerveModuleConstants(
                            DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int DRIVE_MOTOR_ID = 9;
            public static final int ANGLE_MOTOR_ID = 10;
            public static final int CAN_CODER_ID = 3;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(258.13 + 90);
            public static final SwerveModuleConstants CONSTANTS =
                    new SwerveModuleConstants(
                            DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }
    }

    public static final class Arm {
        public static final double MIN_ROT = Units.degreesToRadians(0);
        public static final double MAX_ROT = Units.degreesToRadians(90);

        public static final double STARTING_POS = 0; // TODO CONFIG
        public static final double ABS_ENCODER_OFFSET = 0; // TODO CONFIG

        // TODO CONFIG
        public static final double ROTATION_VELOCITY = Units.degreesToRadians(30);
        public static final double ROTATION_ACCELERATION = Units.degreesToRadians(60);

        public static final int ARM_ROT_LEADER = 18; // Right //TODO CONFIG
        public static final int ARM_ROT_FOLLOWER = 19; // Left //TODO CONFIG

        public static final int ABS_ENCODER = 1; // TODO CONFIG

        public static final int TOP_LIMIT_SWITCH = 2; // TODO CONFIG
        public static final int BOTTOM_LIMIT_SWITCH = 3; // TODO CONFIG

        public static final double RAMP_RATE = 2;

        public static final class PidConstants { // TODO CONFIG
            public static final double P = 0;
            public static final double I = 0;
            public static final double D = 0;
        }

        public static final double GEAR_RATIO = (20.0 / 1.0) * (62.0 / 12.0); // TODO CONFIG
    }

    public static class Intake {
        public static final int CURRENT_LIMIT = 60;

        public static final class Spark {
            public static final int MOTOR_ID = 15;
        }
    }

    public static class Shooter {

        public static class Spark {

        public static final int SPARK_TOP = 22; // TODO SET
        public static final int SPARK_BOTTOM = 23; // TODO SET

        public static final int PID_TOP_P = 0; // TODO SET
        public static final int PID_TOP_I = 0; // TODO SET
        public static final int PID_TOP_D = 0; // TODO SET

        public static final int PID_BOTTOM_P = 0; // TODO SET
        public static final int PID_BOTTOM_I = 0; // TODO SET
        public static final int PID_BOTTOM_D = 0; // TODO SET

        }

        public static class Talon {

        public static final int TALON_TOP = 22; //TODO SET
        public static final int TALON_BOTTOM = 23; //TODO SET    
        }

        public static final int CURRENT_LIMIT = 0; // TODO SET

    }

    public static final class AutoConstants {

        public static final double CHARGE_STATION_MIN_ANGLE = 2;
        public static final double CHARGE_STATION_MAX_ANGLE = 10;

        public static final double MAX_SPEED_METERS_PER_SECOND = 3;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = Math.PI;

        public static final double PX_CONTROLLER = 1;
        public static final double PY_CONTROLLER = 1;
        public static final double P_THETA_CONTROLLER = 1;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
                new TrapezoidProfile.Constraints(
                        MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
                        MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);
    }

    // TODO: Calculate needed deadband for controller (should be like 6% or less)
    // Ran this in the pit; had issues with 10% upped to 12%
    // Have you not seen the comment above, anyways we could probably just shift the deadband or
    // something
    // to fix the issues.
    public static double STICK_DEADBAND = 0.08;

    public static final class Networking {
        public static final String JETSON_IP_ADDRESS = "10.20.83.130";
        public static final int PORT = 5810;
        public static final int INTERVAL = 5;
    }

    public static final class OIConstants {
        public static final CommandXboxController operatorController = new CommandXboxController(1);

        public static final Trigger intake = operatorController.leftTrigger();
        public static final Trigger shooter = operatorController.rightTrigger();

        public static final DoubleSupplier armSpeed = () -> operatorController.getLeftY() * .2;


        public static final Function<Double, Double> inputCurve = (x) -> .8 * x + .2 * (x * x * x);

        // Choose 1, not both.
        public static final boolean USE_XBOX_CONTROLLER = false;
        public static final boolean USE_SAITEK_CONTROLLER = true;

        public static final int DRIVE_CONTROLLER_PORT = 0;
        public static final int BUTTON_BOX_PORT = 1;
    }
}
