package frc.robot.drivetrain;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;

public class Vision {
   
    AprilTagFieldLayout aprilTagFieldLayout;
    PhotonCamera cam;
    PhotonPoseEstimator poseEstimator;

    public Vision(String cameraName)
    {
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        cam = new PhotonCamera(cameraName);
        poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, Constants.VisionConstants.RobotToCam);
    }

    Optional<EstimatedRobotPose> GetEstimatedVisionPose()
    {
        List<PhotonPipelineResult> res = cam.getAllUnreadResults();

        if(res.size() == 0) return null;

        PhotonPipelineResult latest = res.get(res.size() - 1);

        Optional<EstimatedRobotPose> pose = poseEstimator.update(latest);

        return pose;
    }
}
