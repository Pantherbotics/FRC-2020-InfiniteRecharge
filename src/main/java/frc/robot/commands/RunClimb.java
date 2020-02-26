package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class RunClimb extends CommandBase {
    private Drivetrain kDrive;
    private double power;
    private boolean hookOn;
    private Value shiftOn;

    public RunClimb(Drivetrain drive, double power, boolean hookOn, Value shiftOn) {
        kDrive = drive;
        this.power = power;
        this.hookOn = hookOn;
        this.shiftOn = shiftOn;
    }

    @Override
    public void initialize() {
        kDrive.shiftClimbHook(hookOn);
        kDrive.shiftPTO(shiftOn);
    }

    @Override
    public void execute() {
        kDrive.setVelocity(power, 0);
    }
}