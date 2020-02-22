/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private DigitalInput magSensor;
  private ColorWheel colorWheel;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit()
  {
    this.magSensor = new DigitalInput(9);
    this.colorWheel = new ColorWheel(I2C.Port.kOnboard);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  public String stringifyArray(int[] array)
  {
    String s = "(";
    for (int i = 0; i < array.length; i++)
      s += array[i] + (i == (array.length - 1) ? ")" : ", ");
    return s;
  }

  @Override
  public void teleopInit() {
    this.colorWheel.startCountingColorChanges();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    System.out.println(this.magSensor.get() ? "No Magnet" : "Touching Magnet!");

    int[] data = this.colorWheel.getRawData();
    int currentColor = this.colorWheel.getCurrentColor();
    int countedChanges = this.colorWheel.getColorChangeCount();

    System.out.println("Raw sensor data: " + stringifyArray(data));
    System.out.println("Detected as " + currentColor + " which is '" + this.colorWheel.getColorName(currentColor) + "'");
    System.out.println("There have been " + countedChanges + " changes since teleop init\n");

    // check like this:
    // if currentColor == colorWheel.RED
    // if currentColor == colorWheel.NONE
    // ... for BLUE, GREEN, and YELLOW
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
