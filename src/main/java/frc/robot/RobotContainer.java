/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.util.*;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private Drivetrain kDrivetrain = new Drivetrain();
    private Intake kIntake = new Intake();
    private Feeder kFeeder = new Feeder();
    private Shooter kShooter = new Shooter();
    private Turret kTurret = new Turret();
    private Limelight kLimelight = new Limelight();
    private Cameras cam = new Cameras();

    private TrajectoryConfig config = new TrajectoryConfig(Constants.driveMaxVel, Constants.driveMaxAccel)
        .setKinematics(Constants.dKinematics)
            .addConstraint(
                new DifferentialDriveVoltageConstraint(
                    Constants.driveSimpleFF, 
                    Constants.dKinematics, 
                    10
                )
            );

    public AutoPaths ap = new AutoPaths(kDrivetrain, kIntake, kFeeder, kShooter, kTurret, kLimelight, config);

    private Joystick joy = new Joystick(Constants.joyID);
    private Joystick pJoy = new Joystick(Constants.pJoyID);

    private JoystickButton joyBS = new JoystickButton(joy, 1); //Square
    private JoystickButton joyBX = new JoystickButton(joy, 2); //X
    private JoystickButton joyBC = new JoystickButton(joy, 3); //Circle
    private JoystickButton joyBT = new JoystickButton(joy, 4); //Triangle
    private JoystickButton joyLBump = new JoystickButton(joy, 5); //Left Bumper
    private JoystickButton joyRBump = new JoystickButton(joy, 6); //Right Bumper
    private JoystickButton joyLTrig = new JoystickButton(joy, 7); //Left Trigger
    private JoystickButton joyRTrig = new JoystickButton(joy, 8); //Right Trigger
    private JoystickButton joyBShare = new JoystickButton(joy, 9); //Share Button
    private JoystickButton joyBOption = new JoystickButton(joy, 10); //Option Button
    private JoystickButton joyLB = new JoystickButton(joy, 11); //Left Joystick Button
    private JoystickButton joyRB = new JoystickButton(joy, 12); //Right Joystick Button
    private JoystickButton joyBPS4 = new JoystickButton(joy, 13); //PS4 Button
    private JoystickButton joyBTrack = new JoystickButton(joy, 14); //Trackpad
    private POVButton joyPOVN = new POVButton(joy, 0); //North
    private POVButton joyPOVE = new POVButton(joy, 90); //East
    private POVButton joyPOVS = new POVButton(joy, 180); //South
    private POVButton joyPOVW = new POVButton(joy, 270); //West

    private JoystickButton pJoyBA = new JoystickButton(pJoy, 1); //A
    private JoystickButton pJoyBB = new JoystickButton(pJoy, 2); //B
    private JoystickButton pJoyBX = new JoystickButton(pJoy, 3); //X
    private JoystickButton pJoyBY = new JoystickButton(pJoy, 4); //Y
    private JoystickButton pJoyLBump = new JoystickButton(pJoy, 5);
    private JoystickButton pJoyRBump = new JoystickButton(pJoy, 6);
    private JoystickButton pJoyBBack = new JoystickButton(pJoy, 7); //Back
    private JoystickButton pJoyBStart = new JoystickButton(pJoy, 8); //Start
    private JoystickButton pJoyLB = new JoystickButton(pJoy, 9);
    private JoystickButton pJoyRB = new JoystickButton(pJoy, 10);
    private POVButton pJoyPOVN = new POVButton(pJoy, 0);
    private POVButton pJoyPOVE = new POVButton(pJoy, 90);
    private POVButton pJoyPOVS = new POVButton(pJoy, 180);
    private POVButton pJoyPOVW = new POVButton(pJoy, 270);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        /*
        TrajectoryConfig config = new TrajectoryConfig(Constants.driveMaxVel, Constants.driveMaxAccel)
                .setKinematics(Constants.dKinematics)
                    .addConstraint(
                        new DifferentialDriveVoltageConstraint(
                            Constants.driveSimpleFF, 
                            Constants.dKinematics, 
                            10
                        )
                    );
        System.out.println("\n\n\n\n\n\nYEEEEEEEEEEAAAAAAAHHHHHHHHH\n\n\n\n\n\n");

        ap =  new AutoPaths(kDrivetrain, kIntake, kFeeder, kShooter, kTurret, kLimelight, config);
        System.out.println("\n\n\n\n\n\n\nPRINT\n\n\n\n\n\n\n");
        */
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        cam.enableCameras();
        //Drivetrain
        kDrivetrain.setDefaultCommand(new RunCommand(() -> 
            kDrivetrain.setVelocity(getJoyLeftY(), getJoyRightX()), kDrivetrain //Functional, not tuned
        ));
        //Intake
        joyLTrig.whileHeld(new RunIntake(kIntake, RunIntake.State.GROUND, 0.5), true);
        joyRTrig.whileHeld(new RunIntake(kIntake, RunIntake.State.STATION, 0.0), true);
        joyLBump.whenPressed(new RunIntake(kIntake, RunIntake.State.STOW, 0.0), true);
        joyRBump.whileHeld(new RunIntake(kIntake, RunIntake.State.TACOBELL, -0.5), true);

        //Feeder
        joyBT.whileHeld(new RunFeeder(kFeeder, RunFeeder.Roller.FRONT, 0.6), false)
            .whileHeld(new RunFeeder(kFeeder, RunFeeder.Roller.BACK, 0.6), false);
        joyBS.whileHeld(new RunFeeder(kFeeder, RunFeeder.Roller.VERTICAL, 0.4), false);
        joyBX.whileHeld(new RunFeeder(kFeeder, RunFeeder.Roller.FRONT, -0.3), false)
            .whileHeld(new RunFeeder(kFeeder, RunFeeder.Roller.FRONT, -0.3), false)

            .whileHeld(new RunKicker(kShooter, -0.2), true);

        //Turret
        //  pJoyBX.whileHeld(new RunTurret(kTurret, 1.0), false);
        pJoyBStart.whenPressed(new Aimbot(kTurret, kLimelight), true);
        pJoyPOVN.whenPressed(new RunTurret(kTurret, 0.0), true);
        pJoyPOVE.whenPressed(new RunTurret(kTurret, 90.0), true);
        //pJoyPOVS.whenPressed(new RunTurret(kTurret, 180.0), true);
        pJoyPOVW.whenPressed(new RunTurret(kTurret, -90.0), true);
        pJoyRB.whileHeld(new RunTurret(kTurret, 180*(Math.atan2(-getJoyRightY(), getJoyRightX()))/2*Math.PI-90.0), true);

        //Shooter
        pJoyLBump.whileHeld(new RunShooter(kShooter, 4000.0), false);
        pJoyRBump.whileHeld(new RunKicker(kShooter, 1.0), false);

        //Hood
        pJoyBA.whenPressed(new RunHood(kShooter, 0.5), true);
        pJoyBB.whenPressed(new RunHood(kShooter, 0), true);
        pJoyBY.whenPressed(new RunHood(kShooter, 0.8), true);
        pJoyBX.whenPressed(new RunHood(kShooter, 0.55), true);

        //Climber
        joyBPS4.whileHeld(new RunClimb(kDrivetrain, 0.1, true, Value.kForward))
            .whileHeld(new CancelDrivetrain(kDrivetrain));
        joyBTrack.whileHeld(new RunClimb(kDrivetrain, -0.1, false, Value.kReverse))
            .whileHeld(new CancelDrivetrain(kDrivetrain));
        joyBShare.whileHeld(new RunClimb(kDrivetrain, 0.25, false, Value.kForward))
            .whileHeld(new CancelDrivetrain(kDrivetrain));
        
    }

    public double getJoyLeftX() {
        return Math.abs(joy.getRawAxis(Constants.joyLX)) < Constants.deadband ? 0 : joy.getRawAxis(Constants.joyLX);
    }

    public double getJoyLeftY() {
        return Math.abs(joy.getRawAxis(Constants.joyLY)) < Constants.deadband ? 0 : joy.getRawAxis(Constants.joyLY);
    }

    public double getJoyRightX() {
        return Math.abs(joy.getRawAxis(Constants.joyRX)) < Constants.deadband ? 0 : joy.getRawAxis(Constants.joyRX);
    }

    public double getJoyRightY() {
        return Math.abs(joy.getRawAxis(Constants.joyRY)) < Constants.deadband ? 0 : joy.getRawAxis(Constants.joyRY);
    }

    public void updateSmartDashboard() {
        SmartDashboard.putNumber("DTL Velocity", kDrivetrain.getDriveVel()[0]);
        SmartDashboard.putNumber("DTR Velocity", kDrivetrain.getDriveVel()[1]);

        SmartDashboard.putNumber("Vert Current", kFeeder.getVertCurrent());

        SmartDashboard.putNumber("Shooter Speed", kShooter.getShootSpeed());
        SmartDashboard.putNumber("Shooter Current", kShooter.getCurrent());
        SmartDashboard.putNumber("Hood Pos", kShooter.getHood()[0]);
        SmartDashboard.putNumber("HoodPos again", kShooter.getHood()[1]);

        SmartDashboard.putNumber("Turret Pos Raw", kTurret.getPosRaw());
        SmartDashboard.putNumber("Turret Pos", kTurret.getPos());
        SmartDashboard.putNumber("Turret Velocity", kTurret.getVelocity());
        SmartDashboard.putNumber("Turret Current", kTurret.getCurrent());
        SmartDashboard.putBoolean("Mag Sensor", kTurret.getMagSensor());

        SmartDashboard.putNumber("Lime Pitch", kLimelight.getTarget().pitch);
        SmartDashboard.putNumber("Lime Yaw", kLimelight.getTarget().yaw);
        SmartDashboard.putNumber("Lime Area", kLimelight.getTarget().area);

        SmartDashboard.putBoolean("Hook", kDrivetrain.getHook());
        SmartDashboard.putString("Shifter", kDrivetrain.getPTO().toString());
    }

    public ParallelCommandGroup disabledCommands() {
        return new ParallelCommandGroup(new RunIntake(kIntake, RunIntake.State.GROUND, 0.0),
                                        new RunClimb(kDrivetrain, 0.0, true, Value.kReverse)
        );
    }
}
