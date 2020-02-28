package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Target;
import frc.robot.util.Units;

public class TargetedShot extends CommandBase {
    Shooter kShooter;
    Limelight kLimelight;
    double power, shotPower;
    Target t;

    public TargetedShot(Shooter kShooter, Limelight kLimelight, double shotPower ,double idlePower) {
        hasRequirement(kShooter);
        this.kShooter = kShooter;
        this.kLimelight = kLimelight;
        this.shotPower = shotPower;
        power = idlePower;
    }

    @Override
    public void initialize() {
        kShooter.setShooter(power);
    }

    @Override
    public void execute() {
        t = kLimelight.getTarget();
        kShooter.setKicker(0.75);
        if ((t.area > 0.05) && (Math.abs(t.yaw) < 0.5)) {
            kShooter.setShootVel(shotPower);
        }
        kShooter.setHood(Units.target2Hood(t.area, t.pitch));
    }

    @Override
    public void end(boolean interrupted) {
        kShooter.setShooter(power);
        kShooter.setKicker(0.0);
    }
}