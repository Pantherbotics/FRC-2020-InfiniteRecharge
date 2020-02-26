package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PP extends SubsystemBase {

    private TalonSRX mPP = new TalonSRX(Constants.ppID);
    private DigitalInput colour = new DigitalInput(0);

    private int timeoutMs;

    public PP() {
        mPP.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, timeoutMs);
    }
}