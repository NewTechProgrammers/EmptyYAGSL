package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;

public class Vision extends SubsystemBase {
    private final SwerveDrive swerveDrive;

    public Vision(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
    }

    public void updateRobotPose2D() {
        boolean doRejectUpdate = false;
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

        SmartDashboard.putBoolean("Vision Update Rejected", doRejectUpdate);
        SmartDashboard.putString("Vision Reject Reason", rejectReason);

        if (!doRejectUpdate) {
            Pose2d robotPose = mt1.pose;
            double timestamp = mt1.timestampSeconds;

            // Обновляем позицию только если данные прошли проверку
            swerveDrive.addVisionMeasurement(robotPose, timestamp);
        }
    }

    @Override
    public void periodic() {
        updateRobotPose2D();
    }
}