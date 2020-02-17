package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {

    TalonSRX mTurret = new TalonSRX(Constants.turretID);

    private int turretPos = 0;
    private int timeoutMs = 0;

    public Turret() {
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

    public void setTurretPos(int pos) {
        if (pos < Constants.turretLowBound) { pos = Constants.turretLowBound; }
        else if (pos > Constants.turretUpBound) { pos = Constants.turretUpBound; }
        
        turretPos = pos;
    }

    public int getTurretPos() {
        return turretPos;
    }

    public int getEncoder() {
        return mTurret.getSelectedSensorPosition(0);
    }
}