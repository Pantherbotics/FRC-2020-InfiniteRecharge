package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

@SuppressWarnings("unused")
public class DriveTurn extends CommandBase {
    Drivetrain kDrivetrain;
    double dth, start;

    public DriveTurn(Drivetrain kDrivetrain, double dth) {
        this.kDrivetrain = kDrivetrain;
        this.dth = dth;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        kDrivetrain.setVelocity(0.0, 0.0075 * dth - kDrivetrain.getGyro());
    }

    @Override
    public boolean isFinished() {
        return Math.abs(dth - kDrivetrain.getGyro()) < 2.5;
    }
}