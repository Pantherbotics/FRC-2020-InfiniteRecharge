package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class Aimbot extends CommandBase {
    private Turret kTurret;

    public Aimbot(Turret tourettes) {
        kTurret = tourettes;
    }

    @Override
    public void execute() {
        //Code
    }
}