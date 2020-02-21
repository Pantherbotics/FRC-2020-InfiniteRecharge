package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

public class RunIntake extends CommandBase {

    public enum State {
        GROUND, STATION, STOW, TACOBELL;
    }

    private Intake in;
    private State s;
    private double power;

    public RunIntake(Intake in, State s, double power) {
        addRequirements(in);
        this.in = in;
        this.s = s;
        this.power = power;
    }

    @Override
    public void initialize() {
        switch (s) {
            case GROUND:
                in.actuateMain(true);
                in.actuateSub(true);
                break;
            case STATION:
                in.actuateMain(true);
                in.actuateSub(false);
                break;
            case STOW:
                in.actuateMain(false);
                in.actuateSub(false);
                break;
            case TACOBELL:
                in.actuateMain(false);
                in.actuateSub(true);
                break;
            default:
                System.out.println("???");
        }
    }

    @Override
    public void execute() {
        in.powerRoller(power);
    }

    @Override
    public void end(boolean interrupted) {
        in.powerRoller(0);
    }
}