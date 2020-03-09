package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;
import frc.robot.util.Target;

public class ContinuousTracking extends CommandBase {
    private Turret kTurret;
    private Limelight kLimelight;
    private Target t;
    private Drivetrain kDrivetrain;
    private Pose2d lastPose, currentPose;
    private double deltaTheta, trackingAngle;

    /** WARNING: This doesn't work. Make it work.*/
    public ContinuousTracking(Turret kTurret, Limelight kLimelight, Drivetrain kDrivetrain) {
        hasRequirement(kTurret);
        this.kTurret = kTurret;
        this.kLimelight = kLimelight;
        this.kDrivetrain = kDrivetrain;
    }
    @Override
    public void initialize() {
        /** You first need to know where the LL is relative to the goal. Without it we're just lost*/

        lastPose = kDrivetrain.getPose();
        currentPose = lastPose;
        deltaTheta = lastPose.getRotation().getDegrees() - currentPose.getRotation().getDegrees();
    }

    @Override
    public void execute() {
        /**
         * The basic algorithm goes:
         * (1) Determine where the turret current is relative to the goal. You do this by using the Limelight to center on the goal,
         * then look at where the turret is relative to the robot.
         * (2) Once you know where the turret is, just subtract the difference in rotation between the last pose and the current pose from
         * the turret's angle. Let's call this the trackingAngle.
         * (3) Set the turret angle to the trackingAngle.
         * */
        currentPose = kDrivetrain.getPose();
        deltaTheta = lastPose.getRotation().getDegrees() - currentPose.getRotation().getDegrees();

    }

    @Override
    public boolean isFinished() {
        return false;//true;
    }


}