package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class AutoCommand {
    /*

    private final Timer time = new Timer();
    private RamseteCommand ramComm;
    private Drivetrain kDrivetrain;
    private Intake kIntake;
    private Feeder kFeeder;
    private Shooter kShooter;
    private Turret kTurret;

    public AutoCommand( Trajectory traj,
                        Drivetrain kDrivetrain, 
                        Intake kIntake, 
                        Feeder kFeeder, 
                        Shooter kShooter,
                        Turret kTurret
                        ) {
        this.kDrivetrain = kDrivetrain;
        this.kIntake = kIntake;
        this.kFeeder = kFeeder;
        this.kShooter = kShooter;
        this.kTurret = kTurret;
        
        ramComm = new RamseteCommand(
                traj,
                kDrivetrain::getPose,
                new RamseteController(Constants.ramseteB, Constants.ramseteZeta),
                Constants.ff,
                Constants.dKinematics,
                kDrivetrain::getWheelSpeeds,
                new PIDController(Constants.autoKP, Constants.autoKI, Constants.autoKD),
                new PIDController(Constants.autoKP, Constants.autoKI, Constants.autoKD),
                kDrivetrain::ramseteInput,
                kDrivetrain
        );
    }

    public Command start() {
        time.start();

        Thread subsystemThread = new Thread(() -> {
            
        });

        return ramComm.andThen(() -> kDrivetrain.ramseteInput(0.0, 0.0));
    }
    */
}