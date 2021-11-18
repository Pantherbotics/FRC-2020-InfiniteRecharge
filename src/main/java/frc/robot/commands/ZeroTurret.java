package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

@SuppressWarnings("unused")
public class ZeroTurret extends CommandBase {
    Turret kTurret;

    public ZeroTurret(Turret kTurret) {
        hasRequirement(kTurret);
        this.kTurret = kTurret;
    }

    @Override
    public void initialize() {
        kTurret.setAngle(90.0);
    }

    @Override
    public void execute() {
        if (kTurret.getAngle() > 89.0) {
            kTurret.setAngle(-90.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        kTurret.zeroTurret();
    }

    @Override
    public boolean isFinished() {
        return !kTurret.getMagSensor();
    }
}