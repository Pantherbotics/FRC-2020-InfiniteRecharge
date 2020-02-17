package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;

public class RunFeeder extends CommandBase {

    Feeder feed;
    double powerF, powerB;

    public RunFeeder(Feeder feed, double powerF, double powerB) {
        this.feed = feed;
        this.powerF = powerF;
        this.powerB = powerB;
    }

    @Override
    public void execute() {
        feed.powerFeeder(powerF);
        feed.powerColumn(powerB);
    }

    @Override
    public void end(boolean interrupted) {
        feed.powerFeeder(0);
        feed.powerColumn(0);
    }
}