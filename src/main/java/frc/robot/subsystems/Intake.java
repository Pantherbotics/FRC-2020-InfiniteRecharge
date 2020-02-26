package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    TalonSRX mRoller = new TalonSRX(Constants.rollerID);

    Solenoid bigPiston = new Solenoid(Constants.intakeMainID);
    Solenoid smallPiston = new Solenoid(Constants.intakeSubID);

    public Intake() {

    }

    public void powerRoller(double power) {
        mRoller.set(ControlMode.PercentOutput, -power);
    }

    public void actuateMain(boolean on) {
        bigPiston.set(on);
    }

    public void actuateSub(boolean on) {
        smallPiston.set(on);
    }

    public boolean mainActuated() {
        return bigPiston.get();
    }

    public boolean subActuated() {
        return smallPiston.get();
    }
}