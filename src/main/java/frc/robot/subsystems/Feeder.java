package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@SuppressWarnings("unused")
public class Feeder extends SubsystemBase {
    TalonSRX mVertRollers = new TalonSRX(Constants.vertRollersID);
    TalonSRX mFrontBotRollers = new TalonSRX(Constants.frontBotRollersID);
    TalonSRX mBackBelt = new TalonSRX(Constants.backBeltID);

    public Feeder() {

    }

    //The funnel is the bottom and vertical rollers that lead into the column
    public void powerFeeder(double power) {
        mVertRollers.set(ControlMode.PercentOutput, power);
        mFrontBotRollers.set(ControlMode.PercentOutput, -power);
    }

    public void powerColumn(double power) {
        mFrontBotRollers.set(ControlMode.PercentOutput, -power);
        mBackBelt.set(ControlMode.PercentOutput, power);
    }

    public void powerVertical(double power) {
        mVertRollers.set(ControlMode.PercentOutput, power);
    }

    public void powerFront(double power) {
        mFrontBotRollers.set(ControlMode.PercentOutput, -power);
    }

    public void powerBackBelt(double power) {
        mBackBelt.set(ControlMode.PercentOutput, power);
    }

    public double getVertCurrent() {
        return mVertRollers.getStatorCurrent();
    }
}