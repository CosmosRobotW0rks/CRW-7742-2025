package frc.robot.drivetrain;

import static edu.wpi.first.units.Units.Inches;

import java.io.Serial;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.controls.MotionMagicExpoDutyCycle;
import com.fasterxml.jackson.databind.exc.MismatchedInputException;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;

// NWU COORDINATES!!!!
public class SwerveSubsystem extends SubsystemBase {

    XboxController joy;
    
    StructArrayPublisher<SwerveModuleState> swerveStatePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("SwerveStates", SwerveModuleState.struct).publish();
    StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault().getStructTopic("ChassisPose", Pose2d.struct).publish();
    Field2d odomDisplay = new Field2d();

    public AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    SwerveModule[] modules = new SwerveModule[] {
        new SwerveModule(Constants.SwerveConstants.AngleCANID_FL, Constants.SwerveConstants.DriveCANID_FL),
        new SwerveModule(Constants.SwerveConstants.AngleCANID_FR, Constants.SwerveConstants.DriveCANID_FR),
        new SwerveModule(Constants.SwerveConstants.AngleCANID_BL, Constants.SwerveConstants.DriveCANID_BL),
        new SwerveModule(Constants.SwerveConstants.AngleCANID_BR, Constants.SwerveConstants.DriveCANID_BR)
    };

    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(Constants.SwerveConstants.ModuleOffsetM_X, Constants.SwerveConstants.ModuleOffsetM_Y),
            new Translation2d(Constants.SwerveConstants.ModuleOffsetM_X, -Constants.SwerveConstants.ModuleOffsetM_Y),
            new Translation2d(-Constants.SwerveConstants.ModuleOffsetM_X, Constants.SwerveConstants.ModuleOffsetM_Y),
            new Translation2d(-Constants.SwerveConstants.ModuleOffsetM_X, -Constants.SwerveConstants.ModuleOffsetM_Y));

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);


    SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, GetRobotHeading(), GetModulePositions());

    VisionPoseEstimator poseEstimator = new VisionPoseEstimator("AACAM");

    public SwerveSubsystem(XboxController joy)
    {
        this.joy = joy;   
        
        SmartDashboard.putData("Field", odomDisplay);

        SmartDashboard.putData("HOME", Commands.runOnce(() -> {
            for (SwerveModule module : modules) {
                Rotation2d currentAngle = module.GetAngle();

                Rotation2d targetAngle = Rotation2d.fromRadians(currentAngle.getRadians() % (2*Math.PI));

                module.SetTargetAngle(targetAngle);
                System.out.printf("Module Angle: %f", targetAngle.getDegrees());
            }

        }));
    }

    
    void ApplyChassisSpeeds(ChassisSpeeds speeds)
    {

        SmartDashboard.putNumber("AAAAAA", 1000);

        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

        for (int i = 0; i < moduleStates.length; i++)
        {
            SwerveModuleState state = moduleStates[i];
            SwerveModule module = modules[i];

            state.speedMetersPerSecond *= state.angle.minus(module.GetAngle()).getCos(); // cosine compensation

            module.SetTargetState(state);
        }
    }
    void UpdateOdometry()
    {
        Pose2d robotPose = odometry.update(GetRobotHeading(), GetModulePositions());
        odomDisplay.setRobotPose(robotPose);
    }

    void ResetOdometryWithVision()
    {
        Optional<EstimatedRobotPose> optPose = poseEstimator.GetEstimatedPose();

        if(optPose == null || !optPose.isPresent()) return;

        EstimatedRobotPose estimatedPose = optPose.get();

        Pose3d pose3d = estimatedPose.estimatedPose;

        SmartDashboard.putNumber("EstimatedPose X", pose3d.getX());
        SmartDashboard.putNumber("EstimatedPose Y", pose3d.getY());
        SmartDashboard.putNumber("EstimatedPose Z", pose3d.getZ());

        estimatedPose.estimatedPose.getX();

        Pose2d pose = new Pose2d(pose3d.getX(), pose3d.getY(), new Rotation2d(pose3d.getRotation().getX(), pose3d.getRotation().getY()));
        // Is the pose correct? TODO: Check Pose3D to Pose2D conversion

        odometry.resetPose(pose);
    }

    public void SetChassisSpeeds(ChassisSpeeds speeds)
    {
        chassisSpeeds = speeds;

        SmartDashboard.putNumberArray("ChassisSpeeds", new Double[] {speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond});
    }

    public ChassisSpeeds GetChassisSpeeds()
    {
        return chassisSpeeds;
    }


    public void SetFieldOrientedChassisSpeeds(double xSpeed, double ySpeed, double rotSpeed)
    {
        Rotation2d heading = GetRobotHeading();
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, heading);
        chassisSpeeds = speeds;
    }

    public Rotation2d GetRobotHeading() {
        return Rotation2d.fromDegrees(360.0-gyro.getFusedHeading());
    }

    public SwerveModuleState[] GetModuleStates()
    {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++)
        {
            states[i] = modules[i].GetState();
        }
        return states;
    }

    public SwerveModulePosition[] GetModulePositions()
    {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++)
        {
            positions[i] = modules[i].GetPosition();
        }
        return positions;
    }

    public Pose2d GetRobotPose()
    {
        return odometry.getPoseMeters();
    }


    @Override
    public void periodic() {
        ResetOdometryWithVision();
        ApplyChassisSpeeds(chassisSpeeds);
        UpdateOdometry();

        swerveStatePublisher.set(GetModuleStates());
        posePublisher.set(GetRobotPose());
        SmartDashboard.putNumber("RobotHeading", GetRobotHeading().getDegrees());

    }
}
