package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class RunClimb extends CommandBase {
    private final Drivetrain kDrivetrain;
    private final double power;
    private final boolean hookStart;
    private final boolean hookEnd;
    private final boolean shiftStart;
    private final boolean shiftEnd;

    public RunClimb(Drivetrain kDrivetrain, double power, boolean hookStart, boolean hookEnd, boolean shiftStart, boolean shiftEnd) {
        this.kDrivetrain = kDrivetrain;
        this.power = power;
        this.hookStart = hookStart;
        this.hookEnd = hookEnd;
        this.shiftStart = shiftStart;
        this.shiftEnd = shiftEnd;
    }

    @Override
    public void initialize() {
        kDrivetrain.shiftClimbHook(hookStart);
        kDrivetrain.shiftPTO(shiftStart);
    }

    @Override
    public void execute() {
        kDrivetrain.setClimbSpeed(power);
    }

    @Override
    public void end(boolean interrupted) {
        kDrivetrain.shiftPTO(shiftEnd);
        kDrivetrain.shiftClimbHook(hookEnd);
    }
}