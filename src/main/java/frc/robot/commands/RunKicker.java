package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class RunKicker extends CommandBase {
    private Shooter kShooter;
    private double power;

    public RunKicker(Shooter kShooter, double power) {
        this.kShooter = kShooter;
        this.power = power;
    }

    @Override
    public void execute() {
        kShooter.setKicker(power);
    }

    @Override
    public void end(boolean interrupted) {
        kShooter.setKicker(0);
    }
}