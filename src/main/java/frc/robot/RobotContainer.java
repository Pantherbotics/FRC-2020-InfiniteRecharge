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
import frc.robot.commands.RunFeeder.Roller;
import frc.robot.commands.RunIntake.State;
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

    private TrajectoryConfig forwardConfig = new TrajectoryConfig(Constants.driveMaxVel, Constants.driveMaxAccel)
        .setKinematics(Constants.dKinematics)
            .addConstraint(
                new DifferentialDriveVoltageConstraint(
                    Constants.driveSimpleFF, 
                    Constants.dKinematics, 
                    10
                )
            ).setReversed(false);
    
    private TrajectoryConfig reverseConfig = new TrajectoryConfig(Constants.driveMaxVel, Constants.driveMaxAccel)
        .setKinematics(Constants.dKinematics)
            .addConstraint(
                new DifferentialDriveVoltageConstraint(
                    Constants.driveSimpleFF, 
                    Constants.dKinematics, 
                    10
                )
            ).setReversed(true);

    public AutoPaths ap = new AutoPaths(kDrivetrain, kIntake, kFeeder, kShooter, kTurret, kLimelight, forwardConfig, reverseConfig);

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
        //cam.enableCameras();
        //Drivetrain
        kDrivetrain.setDefaultCommand(new RunCommand(() -> 
            kDrivetrain.setVelocity(Math.pow(getJoyLeftY(), 5/3), Math.pow(getJoyRightX(), 5/3)), kDrivetrain //Functional, not tuned
        ));
        //Intaking
        joyLTrig.whileHeld(new RunIntake(kIntake, State.GROUND, 1.0, State.GROUND, 0.0), true)
            .whileHeld(new RunFeeder(kFeeder, Roller.VERTICAL, 0.6), false)
            .whileHeld(new RunFeeder(kFeeder, Roller.FRONT, 0.35), false)
            .whileHeld(new RunFeeder(kFeeder, Roller.BACK, 0.35), false)
            .whileHeld(new RunKicker(kShooter, -0.5), true);
        joyRTrig.whileHeld(new RunIntake(kIntake, State.STATION, 0.0, State.STATION, 0.0), true)
            .whileHeld(new RunFeeder(kFeeder, Roller.VERTICAL, 0.6), false)
            .whileHeld(new RunFeeder(kFeeder, Roller.FRONT, 0.35), false)
            .whileHeld(new RunFeeder(kFeeder, Roller.BACK, 0.35), false)
            .whileHeld(new RunKicker(kShooter, -0.5), true);
        joyLBump.whenPressed(new RunIntake(kIntake, State.STOW, 0.0, State.STOW, 0.0), true);
        //COMMIT TACO BELL
        joyRBump.whileHeld(new TACOBELL(kIntake, -0.75), true)
            .whileHeld(new RunFeeder(kFeeder, Roller.FRONT, -0.75), false)
            .whileHeld(new RunFeeder(kFeeder, Roller.BACK, -0.75), false)
            .whileHeld(new RunFeeder(kFeeder, Roller.VERTICAL, -1.0), false)
            .whileHeld(new RunKicker(kShooter, -0.5), true);

        //Feeder
        
        joyPOVN.whileHeld(new RunFeeder(kFeeder, Roller.FRONT, 0.4), false)
            .whileHeld(new RunFeeder(kFeeder, Roller.BACK, 0.45), false);
        pJoyBY.whileHeld(new RunFeeder(kFeeder, Roller.FRONT, 0.5), false)
            .whileHeld(new RunFeeder(kFeeder, Roller.BACK, 0.5), false)
            .whileHeld(new RunFeeder(kFeeder, Roller.VERTICAL, 0.4), false)
            .whileHeld(new RunKicker(kShooter, -0.5), false);
        //joyBS.whileHeld(new RunFeeder(kFeeder, RunFeeder.Roller.VERTICAL, 0.4), false);
        pJoyBA.whileHeld(new RunFeeder(kFeeder, Roller.FRONT, -0.3), false)
            .whileHeld(new RunFeeder(kFeeder, Roller.BACK, -0.3), false)
            .whileHeld(new RunKicker(kShooter, -0.5), true);
        
        //pJoyBX.whileHeld(new RunFeeder(kFeeder, Roller.FRONT, -0.25), false)
          //  .whileHeld(new RunFeeder(kFeeder, Roller.BACK, -0.25), false)
            //.whileHeld(new RunKicker(kShooter, -0.5), true);

        //Turret
        //pJoyBX.whileHeld(new RunTurret(kTurret, 1.0), false);
        //pJoyBStart.whileHeld(new Aimbot(kTurret, kLimelight), true);
        pJoyPOVN.whenPressed(new RunTurret(kTurret, 0.0), true);
        pJoyPOVE.whenPressed(new RunTurret(kTurret, 90.0), true);
        pJoyPOVS.whenPressed(new RunTurret(kTurret, 180.0), true);
        pJoyPOVW.whenPressed(new RunTurret(kTurret, -90.0), true);
        //pJoyRB.whileHeld(new RunTurret(kTurret, 180*(Math.atan2(-getJoyRightY(), getJoyRightX()))/2*Math.PI-90.0), true);

        //Shooting
        /*
        pJoyLBump.whileHeld(new RunShooter(kShooter, 3150.0), false)
            .whileHeld(new RunKicker(kShooter, 0.75))
            .whileHeld(new Aimbot(kTurret, kLimelight))
            .whileHeld(new RunLights(kLimelight, 3), false);
        */
        pJoyLBump.whileHeld(new TargetedShot(kShooter, kLimelight, Constants.bulletShot, 0.5), true)
            .whileHeld(new RunKicker(kShooter, 0.4), true)
            .whileHeld(new Aimbot(kTurret, kLimelight), true)
            .whileHeld(new RunLights(kLimelight, 0), true);
        
        pJoyRBump.whileHeld(new RunFeeder(kFeeder, Roller.FRONT, 0.75), false)
            .whileHeld(new RunFeeder(kFeeder, Roller.BACK, 0.75), false)
            .whileHeld(new RunFeeder(kFeeder, Roller.VERTICAL, 0.4), false);
        pJoyBStart.whileHeld(new RunShooter(kShooter, 2750.0), false);
        //pJoyBStart.whileHeld(new RunShooter(kShooter, 2000.0), false);
        pJoyBBack.whileHeld(new RunShooter(kShooter, 0.0), true);
        pJoyLB.whileHeld(new RunLights(kLimelight, 0), true);
        //pJoyRBump.whileHeld(new RunKicker(kShooter, 0.75), false);
        //pJoyBBack.whileHeld(new TargetedShot(kShooter, kLimelight, 0.5));

        //Hood
        //pJoyBA.whenPressed(new RunHood(kShooter, 0.95), true);
        pJoyBB.whenPressed(new RunHood(kShooter, 0.01), true);
        //pJoyBY.whenPressed(new RunHood(kShooter, 0.75), true);
        pJoyBX.whenPressed(new RunHood(kShooter, -0.01), true);

        //Climber
        joyBPS4.whileHeld(new RunClimb(kDrivetrain, 0.0, true, Value.kReverse, Value.kReverse));
        joyBTrack.whileHeld(new RunClimb(kDrivetrain, 0.5, true, Value.kForward, Value.kReverse))
            .whileHeld(new CancelDrivetrain(kDrivetrain));
        joyBShare.whileHeld(new RunClimb(kDrivetrain, -0.1, false, Value.kForward, Value.kReverse))
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

    public void zeroGyro() {
        kDrivetrain.zeroGyro();
    }

    public void whenDisabled() {
        kDrivetrain.shiftClimbHook(false);
        kDrivetrain.shiftPTO(Value.kReverse);
        kLimelight.setLights(1);
    }

    public void updateSmartDashboard() {
        SmartDashboard.putNumber("DTR Velocity", kDrivetrain.getDriveVel()[0]);
        SmartDashboard.putNumber("DTL Velocity", kDrivetrain.getDriveVel()[1]);
        SmartDashboard.putNumber("Gyro", kDrivetrain.getBoundAngle());

        SmartDashboard.putNumber("Vert Current", kFeeder.getVertCurrent());

        SmartDashboard.putNumber("Shooter Speed", kShooter.getShootSpeed());
        SmartDashboard.putNumber("Shooter Current", kShooter.getCurrent());
        SmartDashboard.putNumber("Hood Pos", kShooter.getHood()[0]);
        SmartDashboard.putNumber("HoodPos again", kShooter.getHood()[1]);
        SmartDashboard.putBoolean("Shooter Ready?", kShooter.isReady(Constants.bulletShot));

        SmartDashboard.putNumber("Turret Pos Raw", kTurret.getPosRaw());
        SmartDashboard.putNumber("Turret Pos", kTurret.getPos());
        SmartDashboard.putNumber("Turret Velocity", kTurret.getVelocity());
        SmartDashboard.putNumber("Turret Current", kTurret.getCurrent());
        SmartDashboard.putBoolean("Mag Sensor", kTurret.getMagSensor());
        SmartDashboard.putNumber("Turret Angle", kTurret.getAngle());

        SmartDashboard.putNumber("Lime Pitch", kLimelight.getTarget().pitch);
        SmartDashboard.putNumber("Lime Yaw", kLimelight.getTarget().yaw);
        SmartDashboard.putNumber("Lime Area", kLimelight.getTarget().area);

        SmartDashboard.putBoolean("Hook", kDrivetrain.getHook());
        SmartDashboard.putString("Shifter", kDrivetrain.getPTO().toString());
    }
}
