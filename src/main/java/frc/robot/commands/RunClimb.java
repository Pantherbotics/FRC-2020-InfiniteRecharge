package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class RunClimb extends CommandBase {
    private final Drivetrain kDrivetrain;
    private final double power;
    private final boolean hookOn;

    public RunClimb(Drivetrain kDrivetrain, double power, boolean hookOn) {
        this.kDrivetrain = kDrivetrain;
        this.power = power;
        this.hookOn = hookOn;
    }

    @Override
    public void initialize() {
        kDrivetrain.shiftClimbHook(hookOn);
        kDrivetrain.shiftPTO(true);
    }

    @Override
    public void execute() {
        kDrivetrain.setClimbSpeed(power);
    }

    @Override
    public void end(boolean interrupted) {
        kDrivetrain.shiftPTO(false);
    }
}