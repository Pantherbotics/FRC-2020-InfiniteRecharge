package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class TACOBELL extends CommandBase {
    private Intake kIntake;
    private double power;

    public TACOBELL(Intake kIntake, double power) {
        this.kIntake = kIntake;
        this.power = power;
    }

    @Override
    public void initialize() {
        kIntake.actuateMain(false);
        kIntake.actuateSub(true);
    }

    @Override
    public void execute() {
        kIntake.powerRoller(power);
    }

    @Override
    public void end(boolean interrupted) {
        kIntake.powerRoller(0.0);
    }
}