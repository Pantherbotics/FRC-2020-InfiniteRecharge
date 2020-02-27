package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class CancelDrivetrain extends CommandBase {
    private Drivetrain kDrivetrain;

    public CancelDrivetrain(Drivetrain kDrivetrain) {
        this.kDrivetrain = kDrivetrain;
    }

    @Override
    public void initialize() {
        kDrivetrain.setCancel(0);
    }

    @Override
    public void end(boolean interrupted) {
        kDrivetrain.setCancel(1);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}