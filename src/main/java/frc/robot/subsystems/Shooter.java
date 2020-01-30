package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Notifier;
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
    private TalonSRX mTurret = new TalonSRX(Constants.turretID);

    private CANEncoder shootEncoder = new CANEncoder(mShootA, EncoderType.kQuadrature, 1);

    private int timeoutMs = 0;
    private int turretPos = 0;

    public Shooter() {
        mShootB.follow(mShootA, true);

        mTurret.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, timeoutMs);
        mTurret.config_kP(0, Constants.turretP, timeoutMs);
        mTurret.config_kI(0, Constants.turretI, timeoutMs);
        mTurret.config_kD(0, Constants.turretD, timeoutMs);
        mTurret.config_kF(0, Constants.turretF, timeoutMs);
        mTurret.configMotionCruiseVelocity(Constants.turretMaxVel, timeoutMs);
        mTurret.configMotionAcceleration(Constants.turretMaxAccel, timeoutMs);

        Notifier turretLoop = new Notifier(() -> {
            mTurret.set(ControlMode.MotionMagic, turretPos);
        });

        turretLoop.startPeriodic(0.02);
    }

    public void setShooter(double speed) {
        mShootA.set(speed);
    }

    public double getShootSpeed() {
        return shootEncoder.getVelocity();
    }

    public void setTurretPos(int pos) {
        if (pos < Constants.turretLowBound) { pos = Constants.turretLowBound; }
        else if (pos > Constants.turretUpBound) { pos = Constants.turretUpBound; }
        
        turretPos = pos;
    }

    public int getTurretPos() {
        return turretPos;
    }
}