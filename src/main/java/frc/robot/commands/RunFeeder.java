package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;

public class RunFeeder extends CommandBase {

    public enum Roller {
        VERTICAL, FRONT, BACK;
    }

    Feeder feed;
    double power;
    Roller place;

    public RunFeeder(Feeder feed, Roller place, double power) {
        this.feed = feed;
        this.place = place;
        this.power = power;
    }

    @Override
    public void execute() {
        switch (place) {
            case VERTICAL:
                feed.powerVertical(power);
                break;
            case FRONT:
                feed.powerFront(power);
                break;
            case BACK:
                feed.powerBackBelt(power);
                break;
            default:
                System.out.println("?????");
        }
    }

    @Override
    public void end(boolean interrupted) {
        switch (place) {
            case VERTICAL:
                feed.powerVertical(0);
                break;
            case FRONT:
                feed.powerFront(0);
                break;
            case BACK:
                feed.powerBackBelt(0);
                break;
            default:
                System.out.println("?????");
        }
    }
}