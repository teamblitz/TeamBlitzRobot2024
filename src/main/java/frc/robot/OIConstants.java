package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

public class OIConstants {

    public static final CommandXboxController DRIVE_CONTROLLER = new CommandXboxController(0);
    public static final CommandXboxController OPERATOR_CONTROLLER = new CommandXboxController(1);
    public static final boolean TEST_CONTROLS = true;
    public static final CommandXboxController TEST_CONTROLLER =
            TEST_CONTROLS ? new CommandXboxController(2) : null;

    public static final Trigger TELEOP = new Trigger(DriverStation::isTeleop);
    public static final Trigger UNBOUND = new Trigger(() -> false);

    public static final Function<Double, Double> INPUT_CURVE = (x) -> .8 * x + .2 * (x * x * x);

    public static final class Drive {
        public static double STICK_DEADBAND = 0.05;

        // Values are in percents, we have full power
        private static final double SPIN_SPEED = .4;
        private static final double SLOW_SPEED = .3;
        public static final double NORMAL_SPEED = .6;
        public static final double FAST_SPEED = 1;

        private static final SlewRateLimiter DRIVE_MULTIPLIER_LIMITER =
                new SlewRateLimiter(.25); // Todo, try without this?
        private static final DoubleSupplier DRIVE_MULTIPLIER =
                () ->
                        NORMAL_SPEED
                                + DRIVE_CONTROLLER.getRightTriggerAxis()
                                        * (SLOW_SPEED - NORMAL_SPEED)
                                + DRIVE_CONTROLLER.getLeftTriggerAxis()
                                        * (FAST_SPEED - NORMAL_SPEED);

        public static final DoubleSupplier X_TRANSLATION =
                () ->
                        INPUT_CURVE.apply(-DRIVE_CONTROLLER.getLeftY())
                                * DRIVE_MULTIPLIER.getAsDouble();
        public static final DoubleSupplier Y_TRANSLATION =
                () ->
                        INPUT_CURVE.apply(-DRIVE_CONTROLLER.getLeftX())
                                * DRIVE_MULTIPLIER.getAsDouble();

        public static final DoubleSupplier ROTATION_SPEED =
                () -> INPUT_CURVE.apply(.4 * DRIVE_CONTROLLER.getRightX());
        public static final DoubleSupplier HEADING_CONTROL =
                () ->
                        Math.hypot(DRIVE_CONTROLLER.getRightY(), DRIVE_CONTROLLER.getRightX()) > .5
                                ? Math.atan2(
                                                -DRIVE_CONTROLLER.getRightY(),
                                                -DRIVE_CONTROLLER.getRightX())
                                        - 90
                                : Double.NaN;

        // Drive on the fly modes
        public static final Trigger RESET_GYRO = DRIVE_CONTROLLER.start();
        public static final Trigger X_BREAK = UNBOUND;
        public static final Trigger COAST = UNBOUND;
        public static final Trigger BRAKE = UNBOUND;
    }

    public static final class Intake {
        public static final Trigger FEED = OPERATOR_CONTROLLER.leftBumper();
        public static final Trigger EJECT = OPERATOR_CONTROLLER.leftTrigger();
    }

    public static final class Shooter {
        public static final Trigger MANUAL_FEED = OPERATOR_CONTROLLER.rightBumper();
        public static final Trigger shooterAmp = UNBOUND;
        public static final Trigger EJECT = OPERATOR_CONTROLLER.rightTrigger();
    }

    public static final class Arm {
        public static final DoubleSupplier MANUAL_ARM_SPEED =
                () -> -OPERATOR_CONTROLLER.getLeftY() * .2;

        public static final Trigger INTAKE = TELEOP.and(OPERATOR_CONTROLLER.a());
        public static final Trigger TRANSIT_STAGE = TELEOP.and(OPERATOR_CONTROLLER.b());

        public static final Trigger SCORE_SPEAKER = TELEOP.and(OPERATOR_CONTROLLER.y());
        public static final Trigger SCORE_AMP = TELEOP.and(OPERATOR_CONTROLLER.x());

        public static final Trigger AUTO_AIM_SPEAKER = TELEOP.and(OPERATOR_CONTROLLER.start());
    }

    public static final class TestMode {
        public static final Trigger zeroAbsEncoders =
                TEST_CONTROLLER.b().and(DriverStation::isTest);

        public static final class SysId {
            public static final class Arm {
                public static final Trigger armTest =
                        new Trigger(DriverStation::isTest).and(TEST_CONTROLLER.povLeft());
                public static final Trigger quasistaticFwd = armTest.and(TEST_CONTROLLER.y());
                public static final Trigger quasistaticRev = armTest.and(TEST_CONTROLLER.x());
                public static final Trigger dynamicFwd = armTest.and(TEST_CONTROLLER.b());
                public static final Trigger dynamicRev = armTest.and(TEST_CONTROLLER.a());
            }

            public static final class Drive {
                public static final Trigger driveTest =
                        new Trigger(DriverStation::isTest).and(TEST_CONTROLLER.povDown());
                public static final Trigger quasistaticFwd = driveTest.and(TEST_CONTROLLER.y());
                public static final Trigger quasistaticRev = driveTest.and(TEST_CONTROLLER.x());
                public static final Trigger dynamicFwd = driveTest.and(TEST_CONTROLLER.b());
                public static final Trigger dynamicRev = driveTest.and(TEST_CONTROLLER.a());
            }
        }
    } // TODO, i kinda forgot about shuffleboard commands
}
