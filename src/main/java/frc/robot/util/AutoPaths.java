package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

@SuppressWarnings("unused")
public class AutoPaths {
    private final Drivetrain kDrivetrain;
    private final Intake kIntake;
    private final Feeder kFeeder;
    private final Shooter kShooter;
    private final Turret kTurret;
    private final Limelight kLimelight;
    private final TrajectoryConfig config;

    public ArrayList<NamedCommand> trajs = new ArrayList<>();

    public AutoPaths(Drivetrain kDrivetrain, Intake kIntake, Feeder kFeeder, Shooter kShooter, Turret kTurret, Limelight kLimelight, TrajectoryConfig config) {
        System.out.println("\n\n\n\n\n\nlmoa\n\n\n\n\n\n\n");
        this.kDrivetrain = kDrivetrain;
        this.kIntake = kIntake;
        this.kFeeder = kFeeder;
        this.kShooter = kShooter;
        this.kTurret = kTurret;
        this.kLimelight = kLimelight;
        this.config = config;
        System.out.println("\n\n\n\n\n\nalbanian citizenship\n\n\n\n\n\n\n");

        trajs.add(
            new NamedCommand(
                "3 Ball Face Right",
                new RunTurret(kTurret, 90.0)
                .andThen(
                    new ParallelDeadlineGroup(
                        new RunTimer(5.0),
                        new RunLights(kLimelight, 3),
                        new Aimbot(kTurret, kLimelight),
                        new AutoTargetedShot(kShooter, kFeeder, kLimelight, Constants.bulletShot, 0.0)
                    )
                )
                .andThen(new DriveDistance(kDrivetrain, Units.i2R(36.0)))
                .andThen(new DriveTurn(kDrivetrain, -90.0))
                .andThen(new DriveDistance(kDrivetrain, Units.i2R(18.0)))
            )
        );

        trajs.add(
            new NamedCommand(
                "3 Ball Back",
                new RunTurret(kTurret, 90.0)
                .andThen(
                    new ParallelDeadlineGroup(
                        new RunTimer(-5.0),
                        new RunLights(kLimelight, 3),
                        new Aimbot(kTurret, kLimelight),
                        new AutoTargetedShot(kShooter, kFeeder, kLimelight, Constants.bulletShot, 0.0)
                    )
                )
                .andThen(
                    new ParallelDeadlineGroup(
                        new RunTimer(1.0),
                        new RunCommand(() -> kDrivetrain.setVelocity(0.5, 0.0),
                        kDrivetrain
                        )    
                    )
                )
            )
        );

        trajs.add(
            new NamedCommand(
                "3 Ball Face Right",
                new RunTurret(kTurret, 90.0)
                .andThen(
                    new ParallelDeadlineGroup(
                        new RunTimer(5.0),
                        new RunLights(kLimelight, 3),
                        new Aimbot(kTurret, kLimelight),
                        new AutoTargetedShot(kShooter, kFeeder, kLimelight, Constants.bulletShot, 0.0)
                    )
                )
                .andThen(new DriveTurn(kDrivetrain, 90.0))
                .andThen(new DriveDistance(kDrivetrain, Units.i2R(18.0)))
            )
        );

        trajs.add(
            new NamedCommand(
                "Ramsete Test",
                new RamseteCommand(
                    TrajectoryGenerator.generateTrajectory(
                        List.of(
                            new Pose2d(0, 0, Rotation2d.fromDegrees(0.0)),
                            new Pose2d(2, 2, Rotation2d.fromDegrees(90.0)),
                            new Pose2d(0, 4, Rotation2d.fromDegrees(180.0))
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
                    kDrivetrain
                )
            )
        );

        trajs.add(
            new NamedCommand(
                "EZ 5 Points",
                    new ParallelDeadlineGroup(
                            new RunTimer(13.0),
                            new RunLights(kLimelight, 0),
                            new Aimbot(kTurret, kLimelight),
                            new AutoTargetedShot(kShooter, kFeeder, kLimelight, Constants.bulletShot, 0.0)
                    ).andThen(
                        new ParallelDeadlineGroup(
                            new RunTimer(1.0),
                            new RunCommand(() -> kDrivetrain.setVelocity(0.15, 0.0), kDrivetrain)
                        )
                    ).andThen(
                        new RunCommand(() -> kDrivetrain.setVelocity(0.0, 0.0), kDrivetrain)
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
