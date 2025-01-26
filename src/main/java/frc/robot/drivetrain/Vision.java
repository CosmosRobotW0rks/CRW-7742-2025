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

public class Vision {
    final Transform3d robotToCam = new Transform3d(new Translation3d(0.37, 0.0, 1.2), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center


    AprilTagFieldLayout aprilTagFieldLayout;
    PhotonCamera cam;
    PhotonPoseEstimator poseEstimator;

    public Vision(String cameraName)
    {
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        cam = new PhotonCamera(cameraName);
        poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, robotToCam);
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
