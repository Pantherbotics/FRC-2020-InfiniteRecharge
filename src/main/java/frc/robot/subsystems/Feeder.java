package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
    TalonSRX mBotRightRoller = new TalonSRX(Constants.botRightRollerID);
    TalonSRX mFrontLeftRoller = new TalonSRX(Constants.frontLeftRollerID);
    TalonSRX mBackBelt = new TalonSRX(Constants.backBeltID);

    public Feeder() {

    }

    //The funnel is the bottom and vertical rollers that lead into the column
    public void powerFeeder(double power) {
        mBotRightRoller.set(ControlMode.PercentOutput, power);
        mFrontLeftRoller.set(ControlMode.PercentOutput, power);
    }

    public void powerColumn(double power) {
        mFrontLeftRoller.set(ControlMode.PercentOutput, power);
        mBackBelt.set(ControlMode.PercentOutput, power);
    }

    public void powerBackBelt(double power) {
        mBackBelt.set(ControlMode.PercentOutput, power);
    }
}