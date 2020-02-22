package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
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

    private CANSparkMax mLeftA = new CANSparkMax(Constants.leftDriveAID, MotorType.kBrushless);
    private CANSparkMax mLeftB = new CANSparkMax(Constants.leftDriveBID, MotorType.kBrushless);
    private CANSparkMax mRightA = new CANSparkMax(Constants.rightDriveAID, MotorType.kBrushless);
    private CANSparkMax mRightB = new CANSparkMax(Constants.rightDriveBID, MotorType.kBrushless);

    private CANEncoder leftEncoder = mLeftA.getEncoder(EncoderType.kHallSensor, 1);
    private CANEncoder rightEncoder = mRightA.getEncoder(EncoderType.kHallSensor, 1);
    private CANEncoder LBEncoder = mLeftB.getEncoder(EncoderType.kHallSensor, 1);
    private CANEncoder RBEncoder = mRightB.getEncoder(EncoderType.kHallSensor, 1);

    private Solenoid climbHook = new Solenoid(Constants.climbHookID);
    private DoubleSolenoid ptoShifter = new DoubleSolenoid(Constants.ptoForwardID, Constants.ptoReverseID);

    private AHRS gyro = new AHRS(I2C.Port.kOnboard);

    private int driveID = 0;
    private int climbID = 1;

    private double lastPos, currentPos, dPos, x, y, theta;

    public Drivetrain() {
        mLeftA.getPIDController().setP(Constants.driveP, driveID);
        mLeftA.getPIDController().setI(Constants.driveI, driveID);
        mLeftA.getPIDController().setD(Constants.driveD, driveID);
        mLeftA.getPIDController().setFF(Constants.driveFF, driveID);

        mRightA.getPIDController().setP(Constants.driveP, driveID);
        mRightA.getPIDController().setI(Constants.driveI, driveID);
        mRightA.getPIDController().setD(Constants.driveD, driveID);
        mRightA.getPIDController().setFF(Constants.driveFF, driveID);

        mLeftA.getPIDController().setP(Constants.climbP, climbID);
        mLeftA.getPIDController().setI(Constants.climbI, climbID);
        mLeftA.getPIDController().setD(Constants.climbD, climbID);
        
        mRightA.getPIDController().setP(Constants.climbP, climbID);
        mRightA.getPIDController().setI(Constants.climbI, climbID);
        mRightA.getPIDController().setD(Constants.climbD, climbID);

        mLeftA.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mLeftB.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mRightA.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mRightB.setIdleMode(CANSparkMax.IdleMode.kCoast);

        mLeftA.setInverted(true);
        mLeftB.setInverted(true);
        mRightA.setInverted(false);
        mRightB.setInverted(false);

        mLeftB.follow(mLeftA);
        mRightB.follow(mRightA);

        lastPos = 0;
        x = 0;
        y = 0;
        theta = 0;

        Notifier odomLoop = new Notifier(() -> {
            currentPos = (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2;
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
        mLeftA.set(nyoom - zoom);
        mRightA.set(-nyoom - zoom);
    }

    public void setPIDID(int id) {
        //mLeftA.getPIDController().
    }
    
    //Ramsete
    public void ramseteInput(Double left, Double right) {
        mLeftA.setVoltage(left);
        mRightA.setVoltage(right);
    }

    public Pose2d getPose() {
        return new Pose2d(x, y, Rotation2d.fromDegrees(getBoundAngle()));
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(Units.rpm2MPS(leftEncoder.getVelocity()),
                                                Units.rpm2MPS(rightEncoder.getVelocity()));
    }

    //Climber
    public void shiftPTO(DoubleSolenoid.Value val) { //1: Forward, 2: Off, 3: Reverse
        ptoShifter.set(val);
    }

    public void shiftClimbHook(boolean on) {
        climbHook.set(on);
    }

    //Gyroscope
    public double getGyro() {
        return -gyro.getAngle();
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