package frc.robot.util;

@SuppressWarnings("unused")
public class Units {
    public static double rpm2MPS(double rpm) {
        return rpm * 6 * Math.PI / 2.54 / 60;
    }

    public static double mps2RPM(double mps) {
        return mps * 2.54 / 6 / Math.PI * 60;
    }

    public static double fps2RPM(double fps) {
        return fps / 12 / 6 / Math.PI * 60;
    }

    public static double m2R(double meters) {
        return meters * 39.37 / 6 / Math.PI;
    }

    public static int turretAngle2Pos(double angle) {
        return (int) (angle / 180 * 16600);
    }

    public static double turretPos2Angle(int pos) {
        return (double) pos / 16600 * 180;
    }

    public static double i2M(double inch) {
        return inch / 39.37;
    }

    public static double i2R(double inch) {
        return inch / 6 /Math.PI;
    }

    public static double target2Hood(double area, double pitch) {
        return (-0.0079) * pitch + 0.7193; //Linear extrapolation
    }
}