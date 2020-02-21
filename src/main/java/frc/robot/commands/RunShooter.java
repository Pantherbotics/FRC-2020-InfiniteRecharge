package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

public class RunShooter extends CommandBase {

    private Shooter kShooter;
    private double speed;

    public RunShooter(Shooter shoot, double spd) {
        this.kShooter = shoot;
        speed = spd;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        System.out.println("help");
        kShooter.setShooter(speed);
    }

    @Override
    public void end(boolean interrupted) {
        kShooter.setShooter(0);
    }
}