package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;

public class Vision extends SubsystemBase {
    private final SwerveDrive swerveDrive;

    public static boolean doRejectUpdate = false;
    public static boolean twoTags = false;

    public Vision(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
    }

    public void updateRobotPose2D() {
        doRejectUpdate = false;
        twoTags = false;
        String rejectReason = "None"; 

        
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

        
        if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
            if (mt1.rawFiducials[0].ambiguity > 0.7) {
                doRejectUpdate = true;
                rejectReason = "High Ambiguity"; 
            }
            if (mt1.rawFiducials[0].distToCamera > 4) {
                doRejectUpdate = true;
                rejectReason = "Too Far";
            }
        }
        if (mt1.tagCount == 0) {
            doRejectUpdate = true;
            rejectReason = "No Tags";
        }

        twoTags = mt1.tagCount == 2 ? true : false;

        SmartDashboard.putBoolean("Vision Update Rejected", doRejectUpdate);
        SmartDashboard.putString("Vision Reject Reason", rejectReason);

        if (!doRejectUpdate) {
            Pose2d robotPose = mt1.pose;
            double timestamp = mt1.timestampSeconds;

            // Update'ujemy pozycje tylko wtedy, kiedy dane sa ok
            swerveDrive.addVisionMeasurement(robotPose, timestamp);
        }
    }

    @Override
    public void periodic() {
        updateRobotPose2D();
    }
}