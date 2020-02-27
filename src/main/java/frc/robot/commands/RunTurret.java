package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class RunTurret extends CommandBase {

    Turret tourettes;
    double deg;

    public RunTurret(Turret t, double deg) {
        hasRequirement(t);
        tourettes = t;
        this.deg = deg;
    }

    @Override
    public void initialize() {
        tourettes.setAngle(deg);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}