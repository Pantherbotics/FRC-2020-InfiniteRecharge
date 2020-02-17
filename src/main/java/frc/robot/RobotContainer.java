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
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;


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
    private JoystickButton pJoyLTrig = new JoystickButton(pJoy, 7);
    private JoystickButton pJoyRTrig = new JoystickButton(pJoy, 8);
    private JoystickButton pJoyBBack = new JoystickButton(pJoy, 9); //Back
    private JoystickButton pJoyBStart = new JoystickButton(pJoy, 10); //Start
    private JoystickButton pJoyLB = new JoystickButton(pJoy, 11);
    private JoystickButton pJoyRB = new JoystickButton(pJoy, 12);
    private POVButton pJoyPOVN = new POVButton(pJoy, 0);
    private POVButton pJoyPOVE = new POVButton(pJoy, 90);
    private POVButton pJoyPOVS = new POVButton(pJoy, 180);
    private POVButton pJoyPOVW = new POVButton(pJoy, 270);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
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
        //Drivetrain
        kDrivetrain.setDefaultCommand(new DriveOpen(kDrivetrain, this));
        //Intake
        joyLTrig.whileHeld(new RunIntake(kIntake, RunIntake.State.GROUND, 0.5), true);
        joyRTrig.whileHeld(new RunIntake(kIntake, RunIntake.State.STATION, 0.2), true);
        joyLBump.whenPressed(new RunIntake(kIntake, RunIntake.State.STOW, 0.0), true);

        //Feeder
        pJoyLTrig.whileHeld(new RunFeeder(kFeeder, 0.75, 0.3), true);

        //Turret
        
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

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand(Trajectory traj) {
        //AutoCommand comm = new AutoCommand(traj, kDrivetrain, kIntake, kFeeder, kShooter, kTurret);

        //return comm.start();
        return null;
    }
}
