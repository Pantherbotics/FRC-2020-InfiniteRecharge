package frc.robot.util;

public class Units {
    public static double rpm2MPS(double rpm) {
        return rpm * 6 * Math.PI / 2.54 / 60;
    }

    public static int turretAngle2Pos(double angle) {
        return (int) (angle / 180 * 1);
    }

    public static double turretPos2Angle(int pos) {
        return pos / 1 * 180;
    }
}