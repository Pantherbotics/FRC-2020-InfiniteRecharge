package frc.robot.util;

import java.util.HashMap;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class Path {

    private String name;
    private List<Pose2d> waypoints;
    private HashMap<Double, Command> commTimes;

    public Path(String name, List<Pose2d> waypoints, HashMap<Double, Command> commTimes) {
        this.name = name;
        this.waypoints = waypoints;
        this.commTimes = commTimes;
    }

    public String getName() {
        return name;
    }
}