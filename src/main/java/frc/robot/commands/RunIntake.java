package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class RunIntake extends CommandBase {

    public enum State {
        GROUND, STATION, STOW, TACOBELL
    }

    private final Intake kIntake;
    State s, endState;
    private final double power, endPower;

    public RunIntake(Intake kIntake, State s, double power, State endState, double endPower) {
        addRequirements(kIntake);
        this.kIntake = kIntake;
        this.s = s;
        this.power = power;
        this.endState = endState;
        this.endPower = endPower;
    }

    @Override
    public void initialize() {
        switch (s) {
            case GROUND:
                kIntake.actuateMain(true);
                kIntake.actuateSub(true);
                break;
            case STATION:
                kIntake.actuateMain(true);
                kIntake.actuateSub(false);
                break;
            case STOW:
                kIntake.actuateMain(false);
                kIntake.actuateSub(false);
                break;
            case TACOBELL:
                kIntake.actuateMain(false);
                kIntake.actuateSub(true);
                break;
            default:
                System.out.println("???");
        }
    }

    @Override
    public void execute() {
        kIntake.powerRoller(power);
    }

    @Override
    public void end(boolean interrupted) {
        kIntake.powerRoller(endPower);
        switch (endState) {
            case GROUND:
                kIntake.actuateMain(true);
                kIntake.actuateSub(true);
                break;
            case STATION:
                kIntake.actuateMain(true);
                kIntake.actuateSub(false);
                break;
            case STOW:
                kIntake.actuateMain(false);
                kIntake.actuateSub(false);
                break;
            case TACOBELL:
                kIntake.actuateMain(false);
                kIntake.actuateSub(true);
                break;
            default:
                System.out.println("???");
        }
    }
}