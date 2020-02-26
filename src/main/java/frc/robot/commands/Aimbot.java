package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;
import frc.robot.util.Target;

public class Aimbot extends CommandBase {
    private Turret kTurret;
    private Limelight kLimelight;
    private Target t;

    public Aimbot(Turret tourettes, Limelight lime) {
        hasRequirement(kTurret);
        kTurret = tourettes;
        kLimelight = lime;
    }

    @Override
    public void execute() {
        t = kLimelight.getTarget();/*
        if (kLimelight.getTarget().area < 0.05) {
            kTurret.setAngle(t.yaw < 0 ? 135.0 : -135.0);
        }*/
        System.out.println("AIMBOT BEEP BOOP");
        kTurret.setPower(0.02 * t.yaw);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}