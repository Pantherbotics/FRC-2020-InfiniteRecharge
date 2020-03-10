package frc.robot.util;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.commands.RunFeeder.Roller;
import frc.robot.commands.RunIntake.State;
import frc.robot.subsystems.*;

public class AutoPaths {
    private Drivetrain kDrivetrain;
    private Intake kIntake;
    private Feeder kFeeder;
    private Shooter kShooter;
    private Turret kTurret;
    private Limelight kLimelight;
    private TrajectoryConfig fConfig;
    private TrajectoryConfig rConfig;

    public ArrayList<NamedCommand> trajs = new ArrayList<>();

    public AutoPaths(Drivetrain kDrivetrain, Intake kIntake, Feeder kFeeder, Shooter kShooter, Turret kTurret, Limelight kLimelight, TrajectoryConfig fConfig, TrajectoryConfig rConfig) {
        System.out.println("\n\n\n\n\n\nlmoa\n\n\n\n\n\n\n");
        this.kDrivetrain = kDrivetrain;
        this.kIntake = kIntake;
        this.kFeeder = kFeeder;
        this.kShooter = kShooter;
        this.kTurret = kTurret;
        this.kLimelight = kLimelight;
        this.fConfig = fConfig;
        System.out.println("\n\n\n\n\n\nalbanian citizenship\n\n\n\n\n\n\n");

        trajs.add(
            new NamedCommand(
                "SEND 6",
                new SequentialCommandGroup(
                    new ParallelDeadlineGroup(
                        new RunTimer(6.0),
                        new RunLights(kLimelight, 0),
                        new Aimbot(kTurret, kLimelight),
                        new RunIntake(kIntake, State.STATION, 0.0, State.STOW, 0.0),
                        new AutoTargetedShot(kShooter, kFeeder, kLimelight, Constants.bulletShot, 2000.0)
                    ),
                    new ParallelDeadlineGroup(
                        new RamseteCommand(
                            TrajectoryGenerator.generateTrajectory(
                                new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
                                List.of(
                                    new Translation2d(Units.i2M(100.0), Units.i2M(94.66))
                                ),
                                new Pose2d(Units.i2M(250.0), Units.i2M(94.66), Rotation2d.fromDegrees(0.0)),
                                fConfig),
                            kDrivetrain::getPose,
                            new RamseteController(Constants.ramseteB, Constants.ramseteZeta),
                            Constants.driveSimpleFF,
                            Constants.dKinematics,
                            kDrivetrain::getWheelSpeeds,
                            new PIDController(Constants.autoKP, Constants.autoKI, Constants.autoKD),
                            new PIDController(Constants.autoKP, Constants.autoKI, Constants.autoKD),
                            kDrivetrain::ramseteInput,
                            kDrivetrain
                        ),
                        new RunIntake(kIntake, State.GROUND, 1.0, State.STOW, 0.0),
                        new RunFeeder(kFeeder, Roller.FRONT, 0.6),
                        new RunFeeder(kFeeder, Roller.BACK, 0.6),
                        new RunFeeder(kFeeder, Roller.VERTICAL, 0.5),
                        new RunKicker(kShooter, -0.75)
                    ),
                    new ParallelDeadlineGroup(
                        new RamseteCommand(
                            TrajectoryGenerator.generateTrajectory(
                                new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
                                List.of(),
                                new Pose2d(Units.i2M(-250.0), Units.i2M(-94.66), Rotation2d.fromDegrees(0.0)),
                                rConfig
                            ),
                            kDrivetrain::getPose,
                            new RamseteController(Constants.ramseteB, Constants.ramseteZeta),
                            Constants.driveSimpleFF,
                            Constants.dKinematics,
                            kDrivetrain::getWheelSpeeds,
                            new PIDController(Constants.autoKP, Constants.autoKI, Constants.autoKD),
                            new PIDController(Constants.autoKP, Constants.autoKI, Constants.autoKD),
                            kDrivetrain::ramseteInput,
                            kDrivetrain
                        ),
                        new RunKicker(kShooter, -0.75),
                        new RunFeeder(kFeeder, Roller.FRONT, -0.2),
                        new RunFeeder(kFeeder, Roller.BACK, -0.2)
                    ),
                    new ParallelCommandGroup(
                        new RunLights(kLimelight, 0),
                        new Aimbot(kTurret, kLimelight),
                        new AutoTargetedShot(kShooter, kFeeder, kLimelight, Constants.bulletShot, 2000.0)
                    )
                )
            )
        );

        trajs.add(
            new NamedCommand(
                "SEND 8",
                new SequentialCommandGroup(
                    
                )
            )
        )

        trajs.add(
            new NamedCommand(
                "Ramsete Test",
                new ParallelDeadlineGroup(
                    new RamseteCommand(
                        TrajectoryGenerator.generateTrajectory(
                            new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
                            List.of(
                                new Translation2d(Units.i2M(110.0), Units.i2M(-94.66))
                            ),
                            new Pose2d(Units.i2M(250.0), Units.i2M(-94.66), Rotation2d.fromDegrees(0.0)),
                            fConfig
                        ),
                        kDrivetrain::getPose,
                        new RamseteController(Constants.ramseteB, Constants.ramseteZeta),
                        Constants.driveSimpleFF,
                        Constants.dKinematics,
                        kDrivetrain::getWheelSpeeds,
                        new PIDController(Constants.autoKP, Constants.autoKI, Constants.autoKD),
                        new PIDController(Constants.autoKP, Constants.autoKI, Constants.autoKD),
                        kDrivetrain::ramseteInput,
                        kDrivetrain
                    ),
                    new RunIntake(kIntake, State.GROUND, 0.25, State.STOW, 0.0)
                )
                .andThen(
                    new RamseteCommand(
                        TrajectoryGenerator.generateTrajectory(
                            new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
                            List.of(
                                new Translation2d(Units.i2M(-110.0), Units.i2M(94.66))
                            ),
                            new Pose2d(Units.i2M(-250.0), Units.i2M(94.66), Rotation2d.fromDegrees(0.0))
                            ,
                            rConfig
                        ),
                        kDrivetrain::getPose,
                        new RamseteController(Constants.ramseteB, Constants.ramseteZeta),
                        Constants.driveSimpleFF,
                        Constants.dKinematics,
                        kDrivetrain::getWheelSpeeds,
                        new PIDController(Constants.autoKP, Constants.autoKI, Constants.autoKD),
                        new PIDController(Constants.autoKP, Constants.autoKI, Constants.autoKD),
                        kDrivetrain::ramseteInput,
                        kDrivetrain
                    )
                )
            )
        );
/*
        trajs.add(
            new Path(
                "6 Ball Send",
                toHashMap(
                    List.of(
                        new TimedCommand(
                            0.0,
                            new RunTurret(kTurret, 90.0)
                                .andThen(
                                    new ParallelRaceGroup(
                                        new RunLights(kLimelight, 0),
                                        new Aimbot(kTurret, kLimelight),
                                        new TargetedShot(kShooter, kLimelight, Constants.bulletShot, 0.5),
                                        new RunTimer(2.5)
                                    )
                                )
                        ),
                        new TimedCommand(
                            2.5,
                            new ParallelDeadlineGroup(
                                new RamseteCommand(
                                    TrajectoryGenerator.generateTrajectory(
                                        List.of(
                                            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                                            new Pose2d(Units.i2M(66.91), Units.i2M(100.0), Rotation2d.fromDegrees(90.0)),
                                            new Pose2d(Units.i2M(66.91), Units.i2M(200.0), Rotation2d.fromDegrees(90.0))
                                        ),
                                        config),
                                    kDrivetrain::getPose,
                                    new RamseteController(Constants.ramseteB, Constants.ramseteZeta),
                                    Constants.driveSimpleFF,
                                    Constants.dKinematics,
                                    kDrivetrain::getWheelSpeeds,
                                    new PIDController(Constants.autoKP, Constants.autoKI, Constants.autoKD),
                                    new PIDController(Constants.autoKP, Constants.autoKI, Constants.autoKD),
                                    kDrivetrain::ramseteInput,
                                    kDrivetrain
                                )
                                .andThen(() -> kDrivetrain.setVelPID(0, 0)),
                                new RunIntake(kIntake, State.GROUND, 0.5)
                                    .andThen(new RunIntake(kIntake, State.STOW, 0.0)),
                                new RunFeeder(kFeeder, Roller.FRONT, 0.4),
                                new RunFeeder(kFeeder, Roller.BACK, 0.4),
                                new RunFeeder(kFeeder, Roller.VERTICAL, 0.4)
                            )
                            .andThen(
                                new ParallelRaceGroup(
                                    new RunLights(kLimelight, 0),
                                    new Aimbot(kTurret, kLimelight),
                                    new TargetedShot(kShooter, kLimelight, Constants.bulletShot, 0.0),
                                    new RunTimer(5.0)
                                )
                            )
                        )
                    )
                )
            )
        );

        trajs.add(
            new Path(
                "3 Ball Send",
                toHashMap(
                    List.of(
                        new TimedCommand(
                            0.0,
                            new RunTurret(kTurret, -90.0)
                                .andThen(
                                    new ParallelCommandGroup(
                                        new RunLights(kLimelight, 3),
                                        new Aimbot(kTurret, kLimelight),
                                        new AutoTargetedShot(kShooter, kFeeder, kLimelight, Constants.bulletShot, 0.0)
                                    )
                                )
                                .andThen(new RamseteCommand(
                                    TrajectoryGenerator.generateTrajectory(
                                        List.of(
                                            new Pose2d(0, 0, Rotation2d.fromDegrees(0.0)),
                                            new Pose2d(2, 2, Rotation2d.fromDegrees(90.0))
                                        ),
                                        config
                                    ),
                                    kDrivetrain::getPose,
                                    new RamseteController(Constants.ramseteB, Constants.ramseteZeta),
                                    Constants.driveSimpleFF,
                                    Constants.dKinematics,
                                    kDrivetrain::getWheelSpeeds,
                                    new PIDController(Constants.autoKP, Constants.autoKI, Constants.autoKD),
                                    new PIDController(Constants.autoKP, Constants.autoKI, Constants.autoKD),
                                    kDrivetrain::ramseteInput,
                                    kDrivetrain)
                                )
                        )
                    )
                )
            )
        );
        */
    }
/*
    public Path[] trajs = {
        new Path(
            "6 Ball Send",
            toHashMap(
                List.of(
                    new TimedCommand(
                        0.0,
                        new RunTurret(kTurret, 90.0).andThen(new Aimbot(kTurret, kLimelight))
                    ),
                    new TimedCommand(
                        0.05,
                        new TargetedShot(kShooter, kLimelight, 0.4)
                    ),
                    new TimedCommand(
                        2.5,
                        new RamseteCommand(
                            TrajectoryGenerator.generateTrajectory(
                                List.of(
                                    new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                                    new Pose2d(Units.i2M(66.91), Units.i2M(100.0), Rotation2d.fromDegrees(90.0)),
                                    new Pose2d(Units.i2M(66.91), Units.i2M(200.0), Rotation2d.fromDegrees(90.0))
                                ),
                                config),
                            kDrivetrain::getPose,
                            new RamseteController(Constants.ramseteB, Constants.ramseteZeta),
                            Constants.driveSimpleFF,
                            Constants.dKinematics,
                            kDrivetrain::getWheelSpeeds,
                            new PIDController(Constants.autoKP, Constants.autoKI, Constants.autoKD),
                            new PIDController(Constants.autoKP, Constants.autoKI, Constants.autoKD),
                            kDrivetrain::ramseteInput,
                            kDrivetrain
                        ).andThen(() -> kDrivetrain.setVelPID(0, 0))
                    ),
                    new TimedCommand(
                        3.0,
                        new ParallelRaceGroup(
                            new RunIntake(kIntake, State.GROUND, 0.5),
                            new RunFeeder(kFeeder, Roller.FRONT, 0.4),
                            new RunFeeder(kFeeder, Roller.BACK, 0.4),
                            new RunFeeder(kFeeder, Roller.VERTICAL, 0.4),
                            new RunTimer(0.)
                        )
                    ),
                    new TimedCommand(
                        7.0,
                        new TargetedShot(kShooter, kLimelight, 0.0)
                    )
                )
            ))
    };

    private HashMap<String, Command> toHashMap(List<NamedCommand> l) {
        HashMap<String, Command> hm = new HashMap<>();
        for (NamedCommand nc : l) {
            hm.put(nc.getName(), nc.getCommand());
        }

        return hm;
    }*/
}
