package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveDistance extends CommandBase {
    private final Drivetrain kDrivetrain;
    private final double distance;
    private double start;

    public DriveDistance(Drivetrain kDrivetrain, double distance) {
        this.kDrivetrain = kDrivetrain;
        this.distance = distance;
    }

    @Override
    public void initialize() {
        start = kDrivetrain.getAveragePos();
    }

    @Override
    public void execute() {
        kDrivetrain.setVelocity((distance - kDrivetrain.getAveragePos() - start), 0.0);
    }

    @Override
    public boolean isFinished() {
        return (kDrivetrain.getAveragePos() - start) > distance;
    }

    @Override
    public void end(boolean interrupted) {
        kDrivetrain.setVelocity(0.0, 0.0);
    }
}