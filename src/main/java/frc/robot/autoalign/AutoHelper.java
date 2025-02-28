package frc.robot.autoalign;

import java.util.Optional;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.autoalign.commands.ExecuteWhenNearPosition;
import frc.robot.autoalign.commands.FineAlignCommand;
import frc.robot.drivetrain.SwerveSubsystem;
import frc.robot.shooter.commands.TakeCoralCommand;
import frc.robot.shooter.elevator.ElevatorSubsystem;

public class AutoHelper {

    final int[] blueReefAprTagIDs = { 21,22,17,18,19,20 };
    final int[] redReefAprTagIDs = { 10,9,8,7,6,11 };

    private final SwerveSubsystem swerve;
    private final ElevatorSubsystem elevator;

    AprilTagFieldLayout apriltagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    StructPublisher<Pose2d> origPosePub = NetworkTableInstance.getDefault()
            .getStructTopic("OrigPose", Pose2d.struct).publish();

            StructPublisher<Pose2d> offsetPosePub = NetworkTableInstance.getDefault()
                    .getStructTopic("OffsetPose", Pose2d.struct).publish();

    public AutoHelper(SwerveSubsystem swerveSubsystem, ElevatorSubsystem elevatorSubsystem)
    {
        swerve = swerveSubsystem;
        elevator = elevatorSubsystem;

    }

    public Command AlignToCoralStation(CoralStation cs)
    {
        Pose2d target = GetCoralStationAlignPose(cs);
        return DriveToPose(target, true);
    }

    public Command AlignAndTakeCoral(CoralStation cs)
    {
        Translation2d target = GetCoralStationAlignPose(cs).getTranslation();

        return Commands.defer(() -> AlignToCoralStation(cs).alongWith(WaitUntilNearPosition(target, 1).andThen(new TakeCoralCommand(elevator))), Set.of());
    }

    public Command AlignToReefSide(int sideIndex, ReefAlign ra)
    {
        Pose2d target = GetReefAlignPose(sideIndex, ra);
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

    public boolean NearFieldPosition(Translation2d pos, double tolerance)
    {
        double dist = pos.getDistance(swerve.GetRobotPose().getTranslation());

        return Math.abs(dist) < Math.abs(tolerance);
    }

    public Command ExecuteOnceWhenNearPosition(Translation2d pos, double tolerance, Runnable action)
    {
        return new ExecuteWhenNearPosition(this, pos, tolerance, action, true);
    }

    public Command ExecuteDuringNearPosition(Translation2d pos, double tolerance, Runnable action)
    {
        return new ExecuteWhenNearPosition(this, pos, tolerance, action, false);
    }

    public Command WaitUntilNearPosition(Translation2d pos, double tolerance)
    {
        return new ExecuteWhenNearPosition(this, pos, tolerance, true);
    }


    // Helper functions

    private Pose2d GetCoralStationAlignPose(CoralStation cs)
    {
        int aprTagID = cs == CoralStation.Left ? 13 : 12;

        Pose2d target = GetApriltagPose2D(aprTagID, AutoConstants.CoralStationOffset, Rotation2d.fromDegrees(180));

        return target;
    }

    private Pose2d GetReefAlignPose(int sideIndex, ReefAlign ra)
    {
        int blueAprTagID = blueReefAprTagIDs[sideIndex];
        
        Translation2d offset = AutoConstants.ReefAlignOffsets.get(ra);

        Pose2d target = GetApriltagPose2D(blueAprTagID, offset, Rotation2d.fromDegrees(180));

        return target;
    }

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

        Command cmd = AutoBuilder.pathfindToPose(p2d, constraints).finallyDo(() -> swerve.Stop()).andThen(new FineAlignCommand(swerve, p2d));

        cmd.addRequirements(swerve);

        return cmd;
    }

    private Command DriveToPose(Pose2d p2d, boolean allianceOriented)
    {
        return DriveToPose(p2d, allianceOriented, AutoConstants.MaxDriveSpeed, AutoConstants.MaxDriveAccel, AutoConstants.MaxRotSpeed, AutoConstants.MaxRotAccel);
    }


    public enum CoralStation
    {
        Left,
        Right
    }

    public enum ReefAlign
    {
        Left,
        Center,
        Right
    }
}
