package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Units;

public class Turret extends SubsystemBase {

    TalonSRX mTurret = new TalonSRX(Constants.turretID);

    private int turretPos = 0;
    private int timeoutMs = 0;

    public Turret() {
        mTurret.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, timeoutMs);
        mTurret.config_kP(0, Constants.turretP, timeoutMs);
        mTurret.config_kI(0, Constants.turretI, timeoutMs);
        mTurret.config_kD(0, Constants.turretD, timeoutMs);
        mTurret.config_kF(0, Constants.turretF, timeoutMs);
        mTurret.configMotionCruiseVelocity(Constants.turretMaxVel, timeoutMs);
        mTurret.configMotionAcceleration(Constants.turretMaxAccel, timeoutMs);

        Notifier turretLoop = new Notifier(() -> {
            mTurret.set(ControlMode.MotionMagic, turretPos + Constants.turretOffset);
        });

        turretLoop.startPeriodic(0.02);
    }

    public void setAngle(double degrees) {
        setPos(Units.turretAngle2Pos(degrees));
    }

    public void setPos(int pos) {
        if (pos < Constants.turretLowBound) { pos = Constants.turretLowBound; }
        else if (pos > Constants.turretUpBound) { pos = Constants.turretUpBound; }
        
        turretPos = pos;
    }

    public void setPower(double power) {
        //mTurret.set(ControlMode.PercentOutput, power);
    }

    public int getSetpoint() {
        return turretPos;
    }

    public int getPosRaw() {
        return mTurret.getSelectedSensorPosition(0);
    }

    public int getVelocity() {
        return mTurret.getSelectedSensorVelocity(0);
    }

    public int getPos() {
        return getPosRaw() - Constants.turretOffset;
    }

    public double getAngle() {
        return Units.turretPos2Angle(getPos());
    }

    public double getVoltage() {
        return mTurret.getMotorOutputVoltage();
    }

    public double getCurrent() {
        return mTurret.getStatorCurrent();
    }
}