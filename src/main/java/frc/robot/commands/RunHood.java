package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class RunHood extends CommandBase {
    Shooter kShooter;
    double pos;

    public RunHood(Shooter kShooter, double pos) {
        this.kShooter = kShooter;
        this.pos = pos;
    }

    @Override
    public void initialize() {
        kShooter.setHood(pos);
    }

    @Override
    public void execute() {
        isFinished();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}