package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Target;
import frc.robot.util.Units;

public class AutoTargetedShot extends CommandBase {
    Shooter kShooter;
    Feeder kFeeder;
    Limelight kLimelight;
    double power, shotPower;
    Target t;

    public AutoTargetedShot(Shooter kShooter, Feeder kFeeder, Limelight kLimelight, double shotPower ,double idlePower) {
        hasRequirement(kShooter);
        this.kShooter = kShooter;
        this.kFeeder = kFeeder;
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
        if ((t.area > 0.05) && (Math.abs(t.yaw) < 0.65)) {
            kShooter.setShootVel(shotPower);
        }
        kShooter.setHood(Units.target2Hood(t.area, t.pitch));
        if (kShooter.isReady(2800)) {
            kFeeder.powerFront(0.4);
            kFeeder.powerBackBelt(0.5);
            kShooter.setKicker(1.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        kShooter.setShooter(power);
        kShooter.setKicker(0.0);
        kFeeder.powerFront(0.0);
        kFeeder.powerBackBelt(0.0);
    }
}