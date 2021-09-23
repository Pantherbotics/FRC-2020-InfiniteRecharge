/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

@SuppressWarnings("unused")
public final class Constants {

    //Joystick
    public static final int joyID = 0;
    public static final int pJoyID = 1;
    public static final int joyLX = 0;
    public static final int joyLY = 1;
    public static final int joyRX = 2;
    public static final int joyRY = 5;
    public static final double deadband = 0.05;
    
    //Talon IDs
    public static final int leftDriveAID = 13;
    public static final int leftDriveBID = 14;
    public static final int rightDriveAID = 2;
    public static final int rightDriveBID = 1;
    public static final int rollerID = 6;
    public static final int vertRollersID = 10;
    public static final int frontBotRollersID = 5;
    public static final int backBeltID = 8;
    public static final int shootAID = 12;
    public static final int shootBID = 3;
    public static final int kickID = 11;
    public static final int turretID = 4;
    public static final int ppID = 9;

    //Solenoid IDs
    public static final int ptoForwardID = 1;
    public static final int ptoReverseID = 0;
    public static final int climbHookID = 4;
    public static final int intakeMainID = 2;
    public static final int intakeSubID = 3;

    //Drivetrain
    public static final double driveP = 0.125;
    public static final double driveI = 0.0;
    public static final double driveD = 0.25;
    public static final double driveFF = 0.0;
    public static final double driveMaxVel = 3.6576;
    public static final double driveMaxAccel = 2.4384;
    public static final SimpleMotorFeedforward driveSimpleFF = new SimpleMotorFeedforward(0.279, 0.0686, 0.014);
    public static final DifferentialDriveKinematics dKinematics = new DifferentialDriveKinematics(0.5842);

    //Climber
    public static final double climbP = 0.5;
    public static final double climbI = 0;
    public static final double climbD = 0;
    public static final double climbFF = 0.5;

    //Intake

    //Shooter
    public static final double shootP = 0.000025;
    public static final double shootI = 0;
    public static final double shootD = 0.000;
    public static final double shootFF = 0.00050;
    public static final double shootMaxVel = 4000.0;
    public static final double shootMaxAccel = 1000.0;
    public static final int turretOffset = 0;
    public static final int turretUpBound = 16600;
    public static final int turretLowBound = -16600;
    public static final double turretP = 6.0;
    public static final double turretI = 0;
    public static final double turretD = 0;
    public static final double turretF = 0.01;
    public static final int turretMaxVel = 3000;
    public static final int turretMaxAccel = 5000;
    public static final double aimbotP = 0.15;
    public static final double aimbotD = 2.0;
    public static final double aimbotMax = 0.25;
    public static final double bulletShot = 3350.0;

    //Ramsete
    public static final double ramseteB = 2.0;
    public static final double ramseteZeta = 0.7;
    public static final double autoKP = driveP;
    public static final double autoKI = driveI;
    public static final double autoKD = driveD;
}
