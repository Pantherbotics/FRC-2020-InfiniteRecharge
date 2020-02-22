package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class RunClimb extends CommandBase {
    private Drivetrain kDrive;
    private double power;

    public RunClimb(Drivetrain drive, double power) {
        kDrive = drive;
        this.power = power;
    }

    @Override
    public void initialize() {
        kDrive.shiftClimbHook(true);
    }

    @Override
    public void execute() {
        //kDrive.
    }
}