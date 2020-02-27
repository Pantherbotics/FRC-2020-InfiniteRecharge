package frc.robot.util;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
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
    private TrajectoryConfig config;

    public ArrayList<Path> trajs = new ArrayList<>();

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
                                new RunTimer(4.0)
                            )
                        ),
                        new TimedCommand(
                            7.0,
                            new TargetedShot(kShooter, kLimelight, 0.0)
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
*/
    private HashMap<Double, Command> toHashMap(List<TimedCommand> l) {
        HashMap<Double, Command> hm = new HashMap<>();
        for (TimedCommand tc : l) {
            hm.put(tc.getTime(), tc.getCommand());
        }

        return hm;
    }
}