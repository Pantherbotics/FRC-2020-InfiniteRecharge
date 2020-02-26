package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Target;
import frc.robot.util.Units;

public class TargetedShot extends CommandBase {
    Shooter kShooter;
    Limelight kLimelight;
    double power;
    Target t;

    public TargetedShot(Shooter kShooter, Limelight kLimelight, double idlePower) {
        hasRequirement(kShooter);
        this.kShooter = kShooter;
        this.kLimelight = kLimelight;
        power = idlePower;
    }

    @Override
    public void initialize() {
        //kShooter.setShooter(power);
    }

    @Override
    public void execute() {
        t = kLimelight.getTarget();
        kShooter.setKicker(1.0);
        kShooter.setShootVel(Units.target2Spd(t.area, t.pitch));
    }

    @Override
    public void end(boolean interrupted) {
        //kShooter.setShooter(power);
    }
}