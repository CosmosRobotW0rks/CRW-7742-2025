package frc.robot.auto;

import java.io.Serial;
import java.lang.StackWalker.Option;
import java.security.PublicKey;
import java.util.Dictionary;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.controls.MotionMagicExpoDutyCycle;
import com.fasterxml.jackson.databind.exc.MismatchedInputException;
import com.fasterxml.jackson.databind.node.TreeTraversingParser;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.auto.commands.FineAlignCommand;
import frc.robot.drivetrain.SwerveSubsystem;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification.NotificationLevel;

public class AutoHelper {

    final int[] blueReefAprTagIDs = { 21,22,17,18,19,20 };
    final int[] redReefAprTagIDs = { 10,9,8,7,6,11 };

    private final SwerveSubsystem swerve;

    AprilTagFieldLayout apriltagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    StructPublisher<Pose2d> origPosePub = NetworkTableInstance.getDefault()
            .getStructTopic("OrigPose", Pose2d.struct).publish();

            StructPublisher<Pose2d> offsetPosePub = NetworkTableInstance.getDefault()
                    .getStructTopic("OffsetPose", Pose2d.struct).publish();

    public AutoHelper(SwerveSubsystem swerveSubsystem)
    {
        swerve = swerveSubsystem;
    }


    public Command AlignToCoralStation(CoralStation cs)
    {
        int aprTagID = cs == CoralStation.Left ? 13 : 12;

        Pose2d target = GetApriltagPose2D(aprTagID, AutoConstants.CoralStationOffset, Rotation2d.fromDegrees(180));

        return DriveToPose(target, true);
    }

    public Command AlignToReefSide(int sideIndex, ReefAlign ra)
    {
        int blueAprTagID = blueReefAprTagIDs[sideIndex];
        
        Translation2d offset = AutoConstants.ReefAlignOffsets.get(ra);

        Pose2d target = GetApriltagPose2D(blueAprTagID, offset, Rotation2d.fromDegrees(180));

        return DriveToPose(target, true);
    }

    public Command AlignToClosestReefSide(ReefAlign ra, double maxDistance)
    {
        int sideIndex = GetClosestReefSideIndex(maxDistance);

        if(sideIndex == -1) return null;

        return AlignToReefSide(sideIndex, ra);
    }

    public int GetClosestReefSideIndex(double maxDistance)
    {
        boolean isRed = IsRedAlliance();

        int[] aprTagIDs = isRed ? redReefAprTagIDs : blueReefAprTagIDs;

        Translation2d robotTranslation = swerve.GetRobotPose().getTranslation();

        double[] distances = new double[aprTagIDs.length];

        int index = 0;
        double minDistance = -1;

        for(int i = 0; i < aprTagIDs.length; i++)
        {
            Pose2d pose = GetApriltagPose2D(aprTagIDs[i], new Translation2d(), Rotation2d.fromDegrees(180));

            double dist = Math.abs(pose.getTranslation().getDistance(robotTranslation));

            distances[i] = Math.floor(dist * 100.0) / 100.0;
            
            if(i == 0 || dist < minDistance)
            {
                minDistance = dist;
                index = i;
            }
        }

        SmartDashboard.putNumberArray("DISTANCES", distances);

        if(minDistance > maxDistance) return -1;

        return index;
    }

    public static int indexOfSmallest(int[] array){
        int index = 0;
        int min = array[index];
    
        for (int i = 1; i < array.length; i++){
            if (array[i] <= min){
            min = array[i];
            index = i;
            }
        }
            return index;
    }


    // Helper functions

    private boolean IsRedAlliance()
    {
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;   
    }

    private Pose2d GetApriltagPose2D(int id, Transform2d offset)
    {
        Optional<Pose3d> tagPose = apriltagFieldLayout.getTagPose(id);
        
        Pose2d tagPose2D = tagPose.get().toPose2d();

        tagPose2D.transformBy(offset);

        return tagPose2D.transformBy(offset);
    }

    private Pose2d GetApriltagPose2D(int id)
    {
        return GetApriltagPose2D(id, new Transform2d());
    }

    private Pose2d GetApriltagPose2D(int id, double x, double y)
    {
        return GetApriltagPose2D(id, new Transform2d(new Translation2d(x, y), new Rotation2d()));
    }

    private Pose2d GetApriltagPose2D(int id, Translation2d t2d, Rotation2d rot)
    {
        return GetApriltagPose2D(id, new Transform2d(t2d, rot));
    }

    private Command DriveToPose(Pose2d p2d, boolean allianceOriented, double maxVelocityMPS, double maxAccelerationMPSSq, double maxAngularVelocityRadPerSec, double maxAngularAccelerationRadPerSecSq)
    {
        PathConstraints constraints = new PathConstraints(maxVelocityMPS, maxAccelerationMPSSq, maxAngularVelocityRadPerSec, maxAngularAccelerationRadPerSecSq);
        
        boolean flip = allianceOriented && IsRedAlliance();
        p2d = flip ? FlippingUtil.flipFieldPose(p2d) : p2d;

        Command cmd = AutoBuilder.pathfindToPose(p2d, constraints).andThen(new FineAlignCommand(swerve, p2d));

        cmd.addRequirements(swerve);

        return cmd;
    }

    private Command DriveToPose(Pose2d p2d, boolean allianceOriented)
    {
        return DriveToPose(p2d, allianceOriented, AutoConstants.MaxDriveSpeed, AutoConstants.MaxDriveAccel, AutoConstants.MaxRotSpeed, AutoConstants.MaxRotAccel);
    }


    // NOT TESTED YET!!
    // TODO: Test it
    // TODO: flip if required
    private Command DriveToPath(PathPlannerPath path, boolean allianceOriented, double maxVelocityMPS, double maxAccelerationMPSSq, double maxAngularVelocityRadPerSec, double maxAngularAccelerationRadPerSecSq)
    {

        PathConstraints constraints = new PathConstraints(maxVelocityMPS, maxAccelerationMPSSq, maxAngularVelocityRadPerSec, maxAngularAccelerationRadPerSecSq);

        //boolean flip = allianceOriented && FlipRequired();

        Command cmd = AutoBuilder.pathfindThenFollowPath(path, constraints);

        //cmd = cmd.finallyDo(() -> swerve.Stop());

        cmd.addRequirements(swerve);

        return cmd;
    }

    private Command DriveToPath(PathPlannerPath path, boolean allianceOriented)
    {
        return DriveToPath(path, allianceOriented, AutoConstants.MaxDriveSpeed, AutoConstants.MaxDriveAccel, AutoConstants.MaxRotSpeed, AutoConstants.MaxRotAccel);
    }


    public enum CoralStation
    {
        Left,
        Right
    }

    public enum ReefAlign
    {
        Left,
        Mid,
        Right
    }
}
