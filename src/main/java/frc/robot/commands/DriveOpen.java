package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class DriveOpen extends CommandBase {

    Drivetrain kDrivetrain;
    RobotContainer rc;
    double zoom, nyoom;

    public DriveOpen(Drivetrain kDrivetrain, RobotContainer rc) {
        addRequirements(kDrivetrain);
    }

    @Override
    public void execute() {
        kDrivetrain.setPower(rc.getJoyLeftY(), rc.getJoyRightX());
    }
}