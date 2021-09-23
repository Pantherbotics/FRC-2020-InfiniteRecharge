package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class RunClimb extends CommandBase {
    private final Drivetrain kDrivetrain;
    private final double power;
    private final boolean hookOn;
    private final Value shiftOn, endShift;

    public RunClimb(Drivetrain kDrivetrain, double power, boolean hookOn, Value shiftOn, Value endShift) {
        this.kDrivetrain = kDrivetrain;
        this.power = power;
        this.hookOn = hookOn;
        this.shiftOn = shiftOn;
        this.endShift = endShift;
    }

    @Override
    public void initialize() {
        kDrivetrain.shiftClimbHook(hookOn);
        kDrivetrain.shiftPTO(shiftOn);
    }

    @Override
    public void execute() {
        kDrivetrain.setClimbSpeed(power);
    }

    @Override
    public void end(boolean interrupted) {
        kDrivetrain.shiftPTO(endShift);
    }
}