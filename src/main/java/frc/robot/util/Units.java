package frc.robot.util;

public class Units {
    public static double rpm2MPS(double rpm) {
        return rpm * 6 * Math.PI / 2.54 / 60;
    }
}