package frc.robot.util;

import java.util.HashMap;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class AutoPaths {
    Intake kIntake;
    Feeder kFeeder;
    Shooter kShooter;
    Turret kTurret;

    public AutoPaths(Intake kIntake, Feeder kFeeder, Shooter kShooter, Turret kTurret) {
        this.kIntake = kIntake;
        this.kFeeder = kFeeder;
        this.kShooter = kShooter;
        this.kTurret = kTurret;
    }

    public static Path[] trajs = {
        /*
        new Path(
            "Straight Test",
            List.of(
                new Pose2d(0, 0, new Rotation2d(0),
                new Pose2d(2, 0, new Rotation2d(0)))
            ),
            toHashMap(List.of(
                new TimedCommand(1.0, new RunIntake(kIntake, State.GROUND, 0.5))
            ))          
        )
        */
    };

    private static HashMap<Double, Command> toHashMap(List<TimedCommand> l) {
        HashMap<Double, Command> hm = new HashMap<>();
        for (TimedCommand tc : l) {
            hm.put(tc.getTime(), tc.getCommand());
        }

        return hm;
    }
}