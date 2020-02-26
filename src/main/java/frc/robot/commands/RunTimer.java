package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunTimer extends CommandBase {
    Timer time;
    double dt;

    public RunTimer(double dt) {
        time = new Timer();
        this.dt = dt;
    }

    @Override
    public void initialize() {
        time.start();
    }

    @Override
    public void execute() {
        if (time.get() > dt) {
            isFinished();
        }
    }

    @Override
    public void end(boolean interrupted) {
        time.stop(); //ZA WARUDO
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}