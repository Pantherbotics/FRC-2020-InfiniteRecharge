package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
    TalonSRX mBotRoller = new TalonSRX(Constants.botRollerID);
    TalonSRX mVertRoller = new TalonSRX(Constants.vertRollerID);
    TalonSRX mBelts = new TalonSRX(Constants.beltsID);

    public Feeder() {

    }

    //The funnel is the bottom and vertical rollers that lead into the column
    public void powerBot(double power) {
        mBotRoller.set(ControlMode.PercentOutput, power);
    }

    public void powerVert(double power) {
        mVertRoller.set(ControlMode.PercentOutput, power);
    }

    public void powerBelts(double power) {
        mBelts.set(ControlMode.PercentOutput, power);
    }
}