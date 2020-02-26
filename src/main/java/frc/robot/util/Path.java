package frc.robot.util;

import java.util.HashMap;
import edu.wpi.first.wpilibj2.command.Command;

public class Path {

    private String name;
    private HashMap<Double, Command> comms;

    public Path(String name, HashMap<Double, Command> comms) {
        this.name = name;
        this.comms = comms;
    }

    public String getName() {
        return name;
    }

    public HashMap<Double, Command> getComms() {
        return comms;
    }
}