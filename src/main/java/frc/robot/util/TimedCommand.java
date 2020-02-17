package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;

public class TimedCommand {
    private double time;
    private Command comm;

    public TimedCommand(double time, Command comm) {
        this.time = time;
        this.comm = comm;
    }

    public double getTime() {
        return time;
    }

    public Command getCommand() {
        return comm;
    }
}