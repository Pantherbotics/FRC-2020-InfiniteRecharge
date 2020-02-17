/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

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
    public static final int rollerID = 7;
    public static final int botRightRollerID = 3;
    public static final int frontLeftRollerID = 12;
    public static final int backBeltID = 8;
    public static final int shootAID = 0;
    public static final int shootBID = 15;
    public static final int kickID = 11;
    public static final int turretID = 14;

    //Solenoid IDs
    public static final int ptoForwardID = 0;
    public static final int ptoReverseID = 1;
    public static final int intakeMainID = 2;
    public static final int intakeSubID = 3;

    //Drivetrain
    public static final double driveP = 1;
    public static final double driveI = 0;
    public static final double driveD = 0;
    public static final double ff = 0.5;
    public static final DifferentialDriveKinematics dKinematics = new DifferentialDriveKinematics(0);

    //Intake

    //Shooter
    public static final double shootP = 1;
    public static final double shootI = 0;
    public static final double shootD = 0;
    public static final double shootF = 0;
    public static final int turretUpBound = 0;
    public static final int turretLowBound = 0;
    public static final double turretP = 3.5;
    public static final double turretI = 0;
    public static final double turretD = 0;
    public static final double turretF = 0;
    public static final int turretMaxVel = 1000;
    public static final int turretMaxAccel = 2000;

    //Ramsete
    public static final double ramseteB = 2.0;
    public static final double ramseteZeta = 0.7;
    public static final double autoKP = 1;
    public static final double autoKI = 0;
    public static final double autoKD = 0;
}
