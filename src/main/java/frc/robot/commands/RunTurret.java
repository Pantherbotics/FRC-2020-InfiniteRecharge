package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class RunTurret extends CommandBase {

    Turret tourettes;
    int pos;

    public RunTurret(Turret t, int pos) {
        tourettes = t;
        this.pos = pos;
    }

    @Override
    public void execute() {
        tourettes.setTurretPos(pos);

        isFinished();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}