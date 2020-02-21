package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class RunTurret extends CommandBase {

    Turret tourettes;
    double deg;

    public RunTurret(Turret t, double deg) {
        tourettes = t;
        this.deg = deg;
    }

    @Override
    public void execute() {
        tourettes.setAngle(deg);

        isFinished();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}