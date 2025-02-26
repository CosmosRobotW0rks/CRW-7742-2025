package frc.robot.drivetrain;

import static edu.wpi.first.units.Units.Inches;

import java.io.Serial;
import java.lang.StackWalker.Option;
import java.util.List;
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
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

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
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification.NotificationLevel;

// NWU COORDINATES!!!!
public class SwerveSubsystem extends SubsystemBase {

    StructArrayPublisher<SwerveModuleState> swerveStatePublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("SwerveStates", SwerveModuleState.struct).publish();
    StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("ChassisPose", Pose2d.struct).publish();

            Field2d odomDisplay = new Field2d();
            Field2d intendedPoseDisplay = new Field2d();

    public AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    SwerveModule[] modules = new SwerveModule[] {
            new SwerveModule(SwerveConstants.AngleCANID_FL, SwerveConstants.DriveCANID_FL, SwerveConstants.ABSENCPORTID_FL),
            new SwerveModule(SwerveConstants.AngleCANID_FR, SwerveConstants.DriveCANID_FR, SwerveConstants.ABSENCPORTID_FR),
            new SwerveModule(SwerveConstants.AngleCANID_BL, SwerveConstants.DriveCANID_BL, SwerveConstants.ABSENCPORTID_BL),
            new SwerveModule(SwerveConstants.AngleCANID_BR, SwerveConstants.DriveCANID_BR, SwerveConstants.ABSENCPORTID_BR)
    };

    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(SwerveConstants.ModuleOffsetM_X, SwerveConstants.ModuleOffsetM_Y),
            new Translation2d(SwerveConstants.ModuleOffsetM_X, -SwerveConstants.ModuleOffsetM_Y),
            new Translation2d(-SwerveConstants.ModuleOffsetM_X, SwerveConstants.ModuleOffsetM_Y),
            new Translation2d(-SwerveConstants.ModuleOffsetM_X, -SwerveConstants.ModuleOffsetM_Y));

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);

    SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, GetRobotHeading(),
            GetModulePositions(), new Pose2d(7.572, 4.025, new Rotation2d(0)));

    Vision vision = new Vision("AACAM");
    
    double estimatedRobotHeadingRad = 0;

    double lastPoseUpdate = 0;


    public SwerveSubsystem() {
        SmartDashboard.putData("Estimated Pose", odomDisplay);

        InitializePathPlanner();

        InitializeShortcutButtons();
    }


    // TODO: Implement auto-stop when cs does not update for a cycle
    public void SetChassisSpeeds(ChassisSpeeds speeds) {
        chassisSpeeds = speeds;

        SmartDashboard.putNumberArray("ChassisSpeeds", new Double[] { speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond});
    }

    public void SetFieldOrientedChassisSpeeds(ChassisSpeeds cs) {

        cs = ChassisSpeeds.fromFieldRelativeSpeeds(cs.vxMetersPerSecond, cs.vyMetersPerSecond, cs.omegaRadiansPerSecond, GetRobotHeading());

        SetChassisSpeeds(cs);
    }

    public Command StopCommand()
    {
        return Commands.runOnce(() -> Stop(), this);
    }

    public void Stop()
    {
        SetChassisSpeeds(new ChassisSpeeds(0,0,0));
    }

    public ChassisSpeeds GetRobotRelativeChassisSpeeds() {
        return chassisSpeeds;
    }

    public ChassisSpeeds GetFieldRelativeChassisSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, GetRobotHeading());
    }

    public Rotation2d GetRobotHeading() {
        return Robot.isSimulation() ? Rotation2d.fromRadians(estimatedRobotHeadingRad) : Rotation2d.fromDegrees(360.0 - gyro.getFusedHeading());
    }

    public SwerveModuleState[] GetModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].GetState();
        }
        return states;
    }

    public SwerveModulePosition[] GetModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].GetPosition();
        }
        return positions;
    }

    public Pose2d GetRobotPose() {
        return poseEstimator.getEstimatedPosition();
    }

    
    void InitializeShortcutButtons()
    {
        SmartDashboard.putData("Reset Heading", Commands.runOnce(() -> poseEstimator.resetRotation(Rotation2d.kZero)));

        SmartDashboard.putData("Home Swerve", Commands.runOnce(() -> {
            for (SwerveModule swerveModule : modules) {
                swerveModule.Home();
            }
        }, this));
    }

    void InitializePathPlanner() {
        try {

            RobotConfig config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                    () -> GetRobotPose(),
                    p2d -> poseEstimator.resetPose(p2d),
                    () -> GetRobotRelativeChassisSpeeds(),
                    (ChassisSpeeds, DriveFeedforwards) -> SetChassisSpeeds(ChassisSpeeds),
                    new PPHolonomicDriveController(
                            // Translation PID constants
                            new PIDConstants(1.0, 0.0, 0.0),
                            // Rotation PID constants
                            new PIDConstants(1.0, 0.0, 0.0)
                    ),
                    config,
                    () -> {
                        Optional<Alliance> alliance = DriverStation.getAlliance();
                        return alliance.isPresent() && alliance.get() == Alliance.Red;
                    }, this);

            //PathfindingCommand.warmupCommand().schedule();

        } catch (Exception ex) {
            Elastic.sendNotification(new Elastic.Notification(NotificationLevel.ERROR,
                    "CRITICAL ERROR! PATHPLANNER FAILED TO INITIALIZE", ex.getMessage()));
        }
    }

    void ApplyChassisSpeeds(ChassisSpeeds speeds) {

        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

        for (int i = 0; i < moduleStates.length; i++) {
            SwerveModuleState state = moduleStates[i];
            SwerveModule module = modules[i];

            state.speedMetersPerSecond *= state.angle.minus(module.GetAngle()).getCos(); // cosine compensation

            module.SetTargetState(state);
        }
    }

    void UpdatePoseEstimation() {

        Pose2d robotPose = poseEstimator.update(GetRobotHeading(), GetModulePositions());
        odomDisplay.setRobotPose(robotPose);
    }

    void ResetOdometryWithVision() {
        Optional<EstimatedRobotPose> optPose = vision.GetEstimatedVisionPose();

        if (optPose == null || !optPose.isPresent())
            return;

        EstimatedRobotPose camPose = optPose.get();

        lastPoseUpdate = Timer.getFPGATimestamp();

        poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
    }

    @Override
    public void periodic() {
        ResetOdometryWithVision();
        ApplyChassisSpeeds(chassisSpeeds);
        
        if(Robot.isSimulation())
        {
            for (SwerveModule module : modules) {
                module.UpdateSimPos();
            }

            estimatedRobotHeadingRad += chassisSpeeds.omegaRadiansPerSecond * 0.02;
            estimatedRobotHeadingRad %= 2*Math.PI;
        }
        
        UpdatePoseEstimation();

        swerveStatePublisher.set(GetModuleStates());
        posePublisher.set(GetRobotPose());
        SmartDashboard.putNumber("RobotHeading", GetRobotHeading().getDegrees());
        SmartDashboard.putNumber("Odom Upd Elapsed", Math.floor((Timer.getFPGATimestamp() - lastPoseUpdate) * 100.0) / 100.0);
        
    }

}
