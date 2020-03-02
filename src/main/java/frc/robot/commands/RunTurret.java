package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class RunTurret extends CommandBase {

    Turret tourettes;
    double deg;

    public RunTurret(Turret kTurret, double deg) {
        hasRequirement(kTurret);
        tourettes = kTurret;
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
    public void end(boolean interrupted) {
        System.out.println("\n\n\n\n\n\nTURRET DONE\n\n\n\n\n\n\n");
    }

    @Override
    public boolean isFinished() {
        return Math.abs(tourettes.getAngle() - deg) < 5.0;
    }
}