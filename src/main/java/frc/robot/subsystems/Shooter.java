package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/*
    I profusely apologize for the inveitable failures of this shooter subsystem

    If I get it right though I'm a god

        -Matthew, 1/29/20
*/

public class Shooter extends SubsystemBase {
    private CANSparkMax mShootA = new CANSparkMax(Constants.shootAID, MotorType.kBrushless);
    private CANSparkMax mShootB = new CANSparkMax(Constants.shootBID, MotorType.kBrushless);
    private TalonSRX mKick = new TalonSRX(Constants.kickID);
    private PWM servoA = new PWM(0);
    private PWM servoB = new PWM(1);

    private CANEncoder shootEncoder = mShootA.getEncoder(EncoderType.kHallSensor, 1);
    //private CANEncoder BEncoder = mShootB.getEncoder(EncoderType.kHallSensor, 1);

    public Shooter() {
        mShootB.follow(mShootA, true);
        
        mShootA.getPIDController().setP(Constants.shootP, 0);
        mShootA.getPIDController().setI(Constants.shootI, 0);
        mShootA.getPIDController().setD(Constants.shootD, 0);
        mShootA.getPIDController().setFF(Constants.shootFF, 0);
        mShootA.getPIDController().setSmartMotionMaxVelocity(Constants.shootMaxVel, 0);
        mShootA.getPIDController().setSmartMotionMaxAccel(Constants.shootMaxAccel, 0);
    }

    public void setShooter(double speed) {
        //mShootA.getPIDController().setReference(speed, ControlType.kVelocity);
        mShootA.set(speed);
    }

    public void setShootVel(double speed) {
        mShootA.getPIDController().setReference(speed, ControlType.kSmartVelocity, 0);
    }

    public double getShootSpeed() {
        return shootEncoder.getVelocity();
    }

    public double getCurrent() {
        return mShootA.getOutputCurrent();
    }

    public void setKicker(double power) {
        mKick.set(ControlMode.PercentOutput, -power);
    }

    public void setHood(double pos) {
        servoA.setPosition(pos);
        servoB.setPosition(pos);
    }

    public double[] getHood() {
        return new double[] {servoA.getPosition(), servoB.getPosition()};
    }
}