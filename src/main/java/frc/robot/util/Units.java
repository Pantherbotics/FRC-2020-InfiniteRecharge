package frc.robot.util;

public class Units {
    public static double rpm2MPS(double rpm) {
        return rpm * 6 * Math.PI / 2.54 / 60;
    }

    public static int turretAngle2Pos(double angle) {
        return (int) (angle / 180 * 16600);
    }

    public static double turretPos2Angle(int pos) {
        return pos / 16600 * 180;
    }

    public static double i2M(double inch) {
        return inch / 39.37;
    }

    public static double target2Hood(double area, double pitch) {
        return (-0.0906) * pitch + 0.9236; //Linear extrapolation
    }
}