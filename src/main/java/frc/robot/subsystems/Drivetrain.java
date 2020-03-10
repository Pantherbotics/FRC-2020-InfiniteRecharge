package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Units;

public class Drivetrain extends SubsystemBase {

    private CANSparkMax mRightA = new CANSparkMax(Constants.rightDriveAID, MotorType.kBrushless);
    private CANSparkMax mRightB = new CANSparkMax(Constants.rightDriveBID, MotorType.kBrushless);
    private CANSparkMax mLeftA = new CANSparkMax(Constants.leftDriveAID, MotorType.kBrushless);
    private CANSparkMax mLeftB = new CANSparkMax(Constants.leftDriveBID, MotorType.kBrushless);

    private CANEncoder rightEncoder = mRightA.getEncoder(EncoderType.kHallSensor, 1);
    private CANEncoder leftEncoder = mLeftA.getEncoder(EncoderType.kHallSensor, 1);
    private CANEncoder LBEncoder = mRightB.getEncoder(EncoderType.kHallSensor, 1);
    private CANEncoder RBEncoder = mLeftB.getEncoder(EncoderType.kHallSensor, 1);

    private Solenoid climbHook = new Solenoid(Constants.climbHookID);
    private DoubleSolenoid ptoShifter = new DoubleSolenoid(Constants.ptoForwardID, Constants.ptoReverseID);

    private AHRS gyro = new AHRS(I2C.Port.kOnboard);

    private int driveID = 0;
    private int climbID = 1;
    private int cancel = 1;

    private double lastPos, currentPos, dPos, x, y, theta;

    public Drivetrain() {
        mRightA.getPIDController().setP(Constants.driveP, driveID);
        mRightA.getPIDController().setI(Constants.driveI, driveID);
        mRightA.getPIDController().setD(Constants.driveD, driveID);
        mRightA.getPIDController().setFF(Constants.driveFF, driveID);

        mLeftA.getPIDController().setP(Constants.driveP, driveID);
        mLeftA.getPIDController().setI(Constants.driveI, driveID);
        mLeftA.getPIDController().setD(Constants.driveD, driveID);
        mLeftA.getPIDController().setFF(Constants.driveFF, driveID);
        
        mRightA.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mRightB.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mLeftA.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mLeftB.setIdleMode(CANSparkMax.IdleMode.kCoast);

        mRightA.setInverted(true);
        mLeftA.setInverted(false);

        mRightB.follow(mRightA, false);
        mLeftB.follow(mLeftA, false);

        lastPos = 0;
        x = 0;
        y = 0;
        theta = 0;

        shiftClimbHook(false);
        shiftPTO(Value.kReverse);
        zeroGyro();
        

        Notifier odomLoop = new Notifier(() -> {
            currentPos = (rightEncoder.getPosition() + leftEncoder.getPosition()) / 2;
            dPos = currentPos = lastPos;
            theta = Math.toRadians(getBoundAngle());
            x = dPos * Math.cos(theta);
            y = dPos * Math.sin(theta);

            lastPos = currentPos;
        });

        odomLoop.startPeriodic(0.02);
    }

    //Drive Modes
    public void setVelocity(double zoom, double nyoom) {
        mRightA.set(cancel * (-nyoom - zoom));
        mLeftA.set(cancel * (nyoom - zoom));
        //System.out.println("power");
    }

    public void setVelPID(double zoom, double nyoom) {
        mRightA.getPIDController().setReference(cancel * 15 * Units.fps2RPM(-nyoom - zoom), ControlType.kVelocity, 0);
        mLeftA.getPIDController().setReference(cancel * 15 * Units.fps2RPM(nyoom - zoom), ControlType.kVelocity, 0);
    }

    public void setClimbSpeed(double speed) {
        mRightA.set(speed);
        mLeftA.set(speed);
    }

    public double[] getDrivePos() {
        double[] xd = { rightEncoder.getPosition(), leftEncoder.getPosition() };
        return xd;
    }

    public double getAveragePos() {
        return ( rightEncoder.getPosition() + leftEncoder.getPosition() ) / 2;
    }

    public double[] getDriveVel() {
        double[] xd = { rightEncoder.getVelocity(), leftEncoder.getVelocity() };
        return xd;
    }

    //True programming
    public void setCancel(int lmao) {
        cancel = lmao;
    }
    
    //Ramsete
    public void ramseteInput(Double left, Double right) {
        mRightA.setVoltage(left);
        mLeftA.setVoltage(right);
    }

    @FunctionalInterface
    public interface xd {
        Pose2d ramseteInput(Double l, Double r);
    }

    public synchronized Pose2d getPose() {
        return new Pose2d(x, y, Rotation2d.fromDegrees(getBoundAngle()));
    }

    public double getPos() {
        return (rightEncoder.getPosition() + leftEncoder.getPosition()) / 2;
    }

    
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(Units.rpm2MPS(rightEncoder.getVelocity()),
                                                Units.rpm2MPS(leftEncoder.getVelocity()));
    }

    @FunctionalInterface
    public interface lmao {
        DifferentialDriveWheelSpeeds getWheelSpeeds();
    }
    public DifferentialDriveWheelSpeeds getweeeee() {
        return new DifferentialDriveWheelSpeeds(Units.rpm2MPS(rightEncoder.getVelocity()),
                                                Units.rpm2MPS(leftEncoder.getVelocity()));
    }

    //Climber
    public void shiftPTO(DoubleSolenoid.Value val) { //1: Forward, 2: Off, 3: Reverse
        ptoShifter.set(val);
        if (val == Value.kForward) {
            cancel = 0;
        } else {
            cancel = 1;
        }
    }

    public void shiftClimbHook(boolean on) {
        climbHook.set(on);
    }

    public Value getPTO() {
        return ptoShifter.get();
    }

    public boolean getHook() {
        return climbHook.get();
    }

    //Gyroscope
    public double getGyro() {
        return gyro.getAngle();
    }

    public void zeroGyro() {
        gyro.reset();
    }

    public double getBoundHalfDegrees(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }

    public double getBoundAngle() {
        return getBoundHalfDegrees(getGyro());
    }
}