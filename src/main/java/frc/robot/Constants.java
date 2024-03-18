/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
import java.util.Arrays;
import java.util.List;

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

    public static final Mode SIM_MODE = Mode.SIM;

    public static final boolean TUNING_MODE = true;

    public enum Mode {
        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public enum Robot {
        CompBot, DevBot, SimBot
    }

    public static final Robot robot = Robot.CompBot;

    public static final class Drive {
        public static final int PIGEON_ID = 14;
        public static final boolean USE_PIGEON = true;

        public static final COTSSwerveConstants CHOSEN_MODULE =
                COTSSwerveConstants.SDSMK4i(COTSSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.inchesToMeters(24.75);
        public static final double WHEEL_BASE = Units.inchesToMeters(24.75);
        public static final double WHEEL_CIRCUMFERENCE = CHOSEN_MODULE.wheelCircumference;

        /* Motor Inverts */
        public static final boolean ANGLE_MOTOR_INVERT = CHOSEN_MODULE.angleMotorInvert;
        public static final boolean DRIVE_MOTOR_INVERT = CHOSEN_MODULE.driveMotorInvert;

        /* Module Gear Ratios */
        public static final double DRIVE_GEAR_RATIO = CHOSEN_MODULE.driveGearRatio;
        public static final double ANGLE_GEAR_RATIO = CHOSEN_MODULE.angleGearRatio;

        /* Angle Encoder Invert */
        public static final boolean CAN_CODER_INVERT = CHOSEN_MODULE.canCoderInvert;

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
         * Because of how fuses work, they can sustain current draw above their rated value for short periods of time before tripping, meaning these values do have meaning above 40
         */
        public static final class CurrentLimits {
            public static class Spark {

                public static final int DRIVE_SMART_CURRENT_LIMIT = 65;
                public static final int DRIVE_SECONDARY_CURRENT_LIMIT = 80;
            }

            public static class Kraken {
                public static final int DRIVE_STATOR = 120;
            }
        }

        public static final int ANGLE_SMART_CURRENT_LIMIT = 25;
        public static final int ANGLE_SECONDARY_CURRENT_LIMIT = 40;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        /* Angle Motor PID Values */
        public static final double ANGLE_KP = 0.002;
        public static final double ANGLE_KI = 0.0;
        public static final double ANGLE_KD = 0.0;
        public static final double ANGLE_KF = 0.0; // For now, should remain zero

        /* Drive Motor PID Values */
        /*
         * Some possible gains for kp
         * kp : 0.0016 or more likely 0.028215
         * try both or else just guess and check ig
         * .06 something might, but that is quite high
         */
        public static final double DRIVE_KP = 0.028215;
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        public static final double DRIVE_KF = 0.0; // Same here.

        /* Drive Motor Characterization Values in volts*/
        public static final double DRIVE_KS = (0.19714);
        public static final double DRIVE_KV = (2.6198);
        public static final double DRIVE_KA = (0.59488);

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
         * was taken when we had like nothing on our robot, we can go 10.35 radians per second while
         * spinning
         */
        public static final double MAX_ANGULAR_VELOCITY =
                10.0; // TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final boolean ANGLE_BRAKE_MODE = false;
        public static final boolean DRIVE_BRAKE_MODE = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int DRIVE_MOTOR_ID = 6;
            public static final int ANGLE_MOTOR_ID = 7;
            public static final int CAN_CODER_ID = 2;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(359.077);
            public static final SwerveModuleConstants CONSTANTS =
                    new SwerveModuleConstants(
                            DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int DRIVE_MOTOR_ID = 8;
            public static final int ANGLE_MOTOR_ID = 9;
            public static final int CAN_CODER_ID = 3;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(269.736);
            public static final SwerveModuleConstants CONSTANTS =
                    new SwerveModuleConstants(
                            DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int DRIVE_MOTOR_ID = 10;
            public static final int ANGLE_MOTOR_ID = 11;
            public static final int CAN_CODER_ID = 4;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(1.582);
            public static final SwerveModuleConstants CONSTANTS =
                    new SwerveModuleConstants(
                            DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int DRIVE_MOTOR_ID = 12;
            public static final int ANGLE_MOTOR_ID = 13;
            public static final int CAN_CODER_ID = 5;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(89.253);
            public static final SwerveModuleConstants CONSTANTS =
                    new SwerveModuleConstants(
                            DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }
    }

    public static final class Arm {
        public static final double MIN_ROT = Units.degreesToRadians(0);
        public static final double MAX_ROT = Units.degreesToRadians(90);

        public static final double STARTING_POS =
                Units.degreesToRadians(5.63); // 3.349 degrees, alternativly 5.63
        public static final double ABS_ENCODER_OFFSET = 0; // TODO CONFIG

        // TODO CONFIG
        public static final double ROTATION_VELOCITY = Units.degreesToRadians(120);
        public static final double ROTATION_ACCELERATION = Units.degreesToRadians(200); // prev 240

        public static final int ARM_ROT_LEADER = 16;
        public static final int ARM_ROT_FOLLOWER = 15;

        public static final int ABS_ENCODER = 1; // TODO CONFIG

        public static final int TOP_LIMIT_SWITCH = 2; // TODO CONFIG
        public static final int BOTTOM_LIMIT_SWITCH = 3; // TODO CONFIG

        public static final double RAMP_RATE = 2;

        public static final class FeedForwardConstants {
            public static final double KS = 0.41881;
            public static final double KV = 2.4974;
            public static final double KA = 1.4009;
            public static final double KG = 0.92004;
        }

        public static final class PidConstants { // TODO CONFIG
            public static final double P = 1.1; // 0.19905 from sysid
            public static final double I = 0;
            public static final double D = 0;
        }

        public static final double GEAR_RATIO = ((3 * 3 * 4) / 1.0) * (64.0 / 12.0);

        public static final class Positions {
            public static final double INTAKE = STARTING_POS + Units.degreesToRadians(-2);
            public static final double TRANSIT_STAGE = Units.degreesToRadians(10);
            public static final double TRANSIT_NORMAL = Units.degreesToRadians(60);
            public static final double SCORE_AMP = Units.degreesToRadians(120);
            public static final double SCORE_SPEAKER = Units.degreesToRadians(30);
        }
    }

    public static class Intake {
        public static final int CURRENT_LIMIT = 60;

        public static final class Spark {
            public static final int MOTOR_ID = 19;
        }
    }

    public static class Shooter {

        public static final double MAX_VELOCITY = 23.6;

        public static class Spark {

            public static final int SPARK_TOP = 22; // TODO SET
            public static final int SPARK_BOTTOM = 23; // TODO SET

            public static final double PID_TOP_P = 0.013715; // TODO SET Was 0.013715
            public static final int PID_TOP_I = 0; // TODO SET
            public static final int PID_TOP_D = 0; // TODO SET

            public static final double PID_BOTTOM_P = 0.013715; // TODO SET
            public static final int PID_BOTTOM_I = 0; // TODO SET
            public static final int PID_BOTTOM_D = 0; // TODO SET

            public static final double FF_TOP_KS = 0.043031; // TODO SET
            public static final double FF_TOP_KV = 0.39694; // TODO SET
            public static final double FF_TOP_KA = 0.086385;

            public static final double FF_BOTTOM_KS = 0.043031; // TODO SET
            public static final double FF_BOTTOM_KV = 0.39694; // TODO SET
            public static final double FF_BOTTOM_KA = 0.086385;
        }

        public static class Talon {

            public static final int TALON_TOP = 22; // TODO SET
            public static final int TALON_BOTTOM = 23; // TODO SET
        }

        public static final int CURRENT_LIMIT = 0; // TODO SET

        public static class AutoShootConstants {
            public static final Transform2d botToCenterOfRotation =
                    new Transform2d(
                            Units.inchesToMeters(-(15 - 4)),
                            Units.inchesToMeters(4),
                            Rotation2d.fromRadians(0));
            public static final Transform2d centerOfRotationToShooter =
                    new Transform2d(
                            Units.inchesToMeters(-27),
                            Units.inchesToMeters(4),
                            Rotation2d.fromRadians(0));

            public static final double shootAngleOffset = Units.degreesToRadians(20);

            public static final Pose3d goalPoseBlue =
                    new Pose3d(0.2269, 5.5526, 2.0451, new Rotation3d());
            public static final Pose3d goalPoseRed =
                    new Pose3d(16.3062, 5.5556, 2.0446, new Rotation3d());

            public static final double shootVelocity = 23.6 * .8;
        }
    }

    public static class Climber {
        public static final int LEFT_MOTOR_ID = 17;
        public static final int RIGHT_MOTOR_ID = 18;

        public static final double CURRENT_LIMIT = 100;

        public static final double MAX_EXTENSION = 0; //TODO set

        public static final InvertedValue LEFT_INVERT = InvertedValue.Clockwise_Positive;
        //public static final InvertedValue LEFT_INVERT = InvertedValue.CounterClockwise_Positive;
        
        public static final InvertedValue RIGHT_INVERT = InvertedValue.CounterClockwise_Positive;
        //public static final InvertedValue RIGHT_INVERT = InvertedValue.Clockwise_Positive;




    }
    public static final class AutoConstants {

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

        public static final double MAX_MODULE_SPEED = 3; // M/S

        public static final HolonomicPathFollowerConfig HOLONOMIC_PATH_FOLLOWER_CONFIG =
                new HolonomicPathFollowerConfig(
                        new PIDConstants(PX_CONTROLLER, 0, 0), // Translation constants
                        new PIDConstants(P_THETA_CONTROLLER, 0, 0), // Rotation constants
                        MAX_MODULE_SPEED,
                        Drive.CENTER_TO_MODULE
                                .get(Drive.FL)
                                .getNorm(), // Drive base radius (distance from center to furthest
                        // module)
                        new ReplanningConfig());

        public enum StartingPos {
            LEFT(60),
            RIGHT(-60),
            CENTER(0);

            public final double angle;

            private StartingPos(double angle) {
                this.angle = angle;
            }
        }
    }

    public static final class Networking {
        public static final String JETSON_IP_ADDRESS = "10.20.83.130";
        public static final int PORT = 5810;
        public static final int INTERVAL = 5;
    }
}
