package frc.robot.drivetrain;

import static edu.wpi.first.units.Units.Inches;

import java.io.Serial;

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
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {

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

    public SwerveSubsystem()
    {
        
    }

    void ApplyChassisSpeeds(ChassisSpeeds speeds)
    {
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

        for (int i = 0; i < moduleStates.length; i++)
        {
            SwerveModuleState state = moduleStates[i];
            SwerveModule module = modules[i];

            state.optimize(module.GetAngle());

            state.speedMetersPerSecond *= state.angle.minus(module.GetAngle()).getCos(); // cosine compensation

            module.SetTargetState(state);
        }
    }

    void UpdateOdometry()
    {
        odometry.update(GetRobotHeading(), GetModulePositions());
    }

    public void SetChassisSpeeds(ChassisSpeeds speeds)
    {
        chassisSpeeds = speeds;
    }


    public void SetFieldOrientedChassisSpeeds(double xSpeed, double ySpeed, double rotSpeed)
    {
        Rotation2d heading = GetRobotHeading();
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, heading);
        chassisSpeeds = speeds;
    }

    public Rotation2d GetRobotHeading() {
        return Rotation2d.fromDegrees(-gyro.getFusedHeading());
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
        ApplyChassisSpeeds(chassisSpeeds);
        UpdateOdometry();
    }
}
