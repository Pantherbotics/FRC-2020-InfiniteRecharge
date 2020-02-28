package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;

public class RunLights extends CommandBase {
    private Limelight kLimelight;
    private int state;

    public RunLights(Limelight kLimelight, int state) { //0 for default, 1 for force off
        this.kLimelight = kLimelight;
        this.state = state;
    }

    @Override
    public void initialize() {
        kLimelight.setLights(state);
    }

    @Override
    public void end(boolean interrupted) {
        kLimelight.setLights(1);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}