package frc.lib.math;

public class Angles {
    private Angles() {}

    public static double wrapAnglePi(double degrees) {
        return wrapAngle(degrees, -Math.PI, Math.PI);
    }

    public static double wrapAngle(double radians, double min, double max) {
        double angle = radians % (2*Math.PI);
        if (angle > max) {
            angle -= 2*Math.PI;
        } else if (angle < min) {
            angle += 2*Math.PI;
        }
        return angle;
    }
}
