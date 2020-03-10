package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;

public class TurretToWall extends CommandBase {
    Drivetrain kDrivetrain;
    Turret kTurret;

    public TurretToWall(Drivetrain kDrivetrain, Turret kTurret) {
        this.kDrivetrain = kDrivetrain;
        this.kTurret = kTurret;
    }

    @Override
    public void initialize() {
        kTurret.setAngle(kDrivetrain.getGyro());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}