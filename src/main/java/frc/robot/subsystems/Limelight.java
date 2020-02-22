package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;

public class Limelight
{
    private NetworkTable limelight;
    private NetworkTableEntry limelightTX;
    private NetworkTableEntry limelightTY;
    private NetworkTableEntry limelightTA;

    private Notifier updateLoop;

    private Target currentTarget;

    public Limelight()
    {
        this.currentTarget = new Target();

        this.limelight = NetworkTableInstance.getDefault().getTable("limelight");
        this.limelightTX = this.limelight.getEntry("tx");
        this.limelightTY = this.limelight.getEntry("ty");
        this.limelightTA = this.limelight.getEntry("ta");

        this.updateLoop = new Notifier(() -> {
            this.update();
        });

        this.updateLoop.startPeriodic(20f / 1000f);
    }

    private void update()
    {
        this.currentTarget.pitch = this.limelightTY.getDouble(0);
        this.currentTarget.yaw   = this.limelightTX.getDouble(0);
        this.currentTarget.area  = this.limelightTA.getDouble(0);
    }

    public Target getTarget()
    {
        return this.currentTarget;
    }
}