package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/*
    I profusely apologize for the inevitable failures of this shooter subsystem

    If I get it right though I'm a god

        -Matthew, 1/29/20
*/

@SuppressWarnings("unused")
public class Shooter extends SubsystemBase {
    private final CANSparkMax mShootA = new CANSparkMax(Constants.shootAID, MotorType.kBrushless);
    private final CANSparkMax mShootB = new CANSparkMax(Constants.shootBID, MotorType.kBrushless);
    private final VictorSPX mKick = new VictorSPX(Constants.kickID);
    private final PWM servoA = new PWM(0);
    private final PWM servoB = new PWM(1);

    private final CANEncoder shootEncoder = mShootA.getEncoder(EncoderType.kHallSensor, 1);
    //private CANEncoder BEncoder = mShootB.getEncoder(EncoderType.kHallSensor, 1);

    public Shooter() {
        mShootA.setInverted(true);
        mShootB.follow(mShootA, true);
        
        mShootA.getPIDController().setP(Constants.shootP, 0);
        mShootA.getPIDController().setI(Constants.shootI, 0);
        mShootA.getPIDController().setD(Constants.shootD, 0);
        mShootA.getPIDController().setFF(Constants.shootFF, 0);
        mShootA.getPIDController().setSmartMotionMaxVelocity(Constants.shootMaxVel, 0);
        mShootA.getPIDController().setSmartMotionMaxAccel(Constants.shootMaxAccel, 0);

        setHood(0.725);
    }

    public void setShooter(double speed) {
        //mShootA.getPIDController().setReference(speed, ControlType.kVelocity);
        mShootA.set(speed);
        //System.out.println(speed);
    }

    public void setShootVel(double speed) {
        mShootA.set(speed / 4875); //Jake's Max: 4875         Matt's Max: 4000
        //mShootA.set((speed / 4000.0 + (speed - getShootSpeed()) / 1000) < 1.0 ? (speed / 4000.0 + (speed - getShootSpeed()) / 1000) : 1.0);
    }

    public void setShootVolt(double v) {
        //System.out.println(v);
        mShootA.setVoltage(v * 12.0);
    }

    public double getShootSpeed() {
        return shootEncoder.getVelocity();
    }

    public boolean isReady(double speed) {
        return getShootSpeed() >= speed;
    }

    //public boolean isReady(double speed) {
    //    return Math.abs(speed - getShootSpeed()) < 50;
    //}

    public double getCurrent() {
        return mShootA.getOutputCurrent();
    }

    public void setKicker(double power) {
        mKick.set(ControlMode.PercentOutput, -power);
        //System.out.println(-power);
    }

    public void setHood(double pos) {
        if (pos < 0.2)
            pos = 0.2;
        servoA.setPosition(pos);
        servoB.setPosition(pos);
    }


    public double[] getHood() {
        return new double[] {servoA.getPosition(), servoB.getPosition()};
    }
}