package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;

public class RunFeeder extends CommandBase {

    public enum Roller {
        VERTICAL, FRONT, BACK
    }

    Feeder kFeeder;
    double power;
    Roller place;

    public RunFeeder(Feeder kFeeder, Roller place, double power) {
        this.kFeeder = kFeeder;
        this.place = place;
        this.power = power;
    }

    @Override
    public void execute() {
        switch (place) {
            case VERTICAL:
                kFeeder.powerVertical(power);
                break;
            case FRONT:
                kFeeder.powerFront(power);
                break;
            case BACK:
                kFeeder.powerBackBelt(power);
                break;
            default:
                System.out.println("?????");
        }
    }

    @Override
    public void end(boolean interrupted) {
        switch (place) {
            case VERTICAL:
                kFeeder.powerVertical(0);
                break;
            case FRONT:
                kFeeder.powerFront(0);
                break;
            case BACK:
                kFeeder.powerBackBelt(0);
                break;
            default:
                System.out.println("?????");
        }
    }
}