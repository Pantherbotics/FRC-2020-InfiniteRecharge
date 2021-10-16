package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class RunShiftPTO extends CommandBase {
    private final Drivetrain kDrivetrain;
    private final boolean shift;

    public RunShiftPTO(Drivetrain kDrivetrain, boolean shift) {
        this.kDrivetrain = kDrivetrain;
        this.shift = shift;
    }

    @Override
    public void initialize() {
        kDrivetrain.shiftPTO(shift);
        if (shift) {
            kDrivetrain.setCancel(1);
        }else {
            kDrivetrain.setCancel(0);
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}