package frc.robot.drivetrain;

import static edu.wpi.first.units.Units.Inches;

import java.io.Serial;

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

public class OldSwerveSubsystem extends SubsystemBase{

    // WIDTH: 11.5" / HEIGHT: 10.5"
    
    final double WIDTH = 0.28448; // meters
    final double HEIGHT = 0.2667; // meters

    OldSwerveModule FL = new OldSwerveModule(Constants.SwerveConstants.AngleCANID_FL, Constants.SwerveConstants.DriveCANID_FL);
    OldSwerveModule FR = new OldSwerveModule(Constants.SwerveConstants.AngleCANID_FR, Constants.SwerveConstants.DriveCANID_FR);
    OldSwerveModule BL = new OldSwerveModule(Constants.SwerveConstants.AngleCANID_BL, Constants.SwerveConstants.DriveCANID_BL);
    OldSwerveModule BR = new OldSwerveModule(Constants.SwerveConstants.AngleCANID_BR, Constants.SwerveConstants.DriveCANID_BR);

    StructArrayPublisher<SwerveModuleState> swerveStatePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("SwerveStates", SwerveModuleState.struct).publish();
    StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault().getStructTopic("ChassisPose", Pose2d.struct).publish();

    public AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
    private Rotation2d gyroAngle = Rotation2d.fromDegrees(0);

    Field2d odometryDisplay = new Field2d();

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);

    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(WIDTH, HEIGHT),
            new Translation2d(WIDTH, -HEIGHT),
            new Translation2d(-WIDTH, HEIGHT),
            new Translation2d(-WIDTH, -HEIGHT));

    double lastJoystickUpdateTimestamp = 0;

    public OldSwerveSubsystem() {

        UpdateModules();

        odometry = new SwerveDriveOdometry(kinematics, gyroAngle, positions);
        odometryDisplay.setRobotPose(new Pose2d(new Translation2d(), new Rotation2d()));

        SmartDashboard.putData("Field", odometryDisplay);

    }

    @Override
    public void periodic() {

        gyroAngle = GetGyroAngle();

        UpdateModules();
        UpdateOdometry();
        Drive();
        Display();
    }

    // Basic Control

    public void SetTargetAbsolute(double fl, double fr, double bl, double br, boolean forceForward) {
        FL.SetTargetAngle(fl, forceForward);
        FR.SetTargetAngle(fr, forceForward);
        BL.SetTargetAngle(bl, forceForward);
        BR.SetTargetAngle(br, forceForward);
    }

    public void SetTargetAbsolute(double tl, double tr, double bl, double br) {
        SetTargetAbsolute(tl, tr, bl, br, false);
    }



    public void SetChassisSpeeds(ChassisSpeeds cs)
    {
        chassisSpeeds = cs;
    }

    public ChassisSpeeds GetChassisSpeeds()
    {
        return chassisSpeeds;
    }

    public void DriveFieldOriented(double x, double y, double rot) {
        SetChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, gyroAngle));

        SmartDashboard.putNumber("FO Input X", x);
        SmartDashboard.putNumber("FO Input Y", y);
        SmartDashboard.putNumber("FO Input Rot", rot);
    }


    // DRIVE & ODOMETRY

    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    SwerveModuleState[] states = new SwerveModuleState[4];
    SwerveDriveOdometry odometry;
    Pose2d OdometryOutPose;


    void Drive() {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
        swerveStatePublisher.set(states);

        SmartDashboard.putNumber("MODULE_SPEED", states[0].speedMetersPerSecond);
        SmartDashboard.putNumber("MODULE_ANGLE", states[0].angle.getDegrees());

        odometry.update(gyroAngle, positions);

        FL.Drive(states[0].speedMetersPerSecond);
        FR.Drive(states[1].speedMetersPerSecond);
        BL.Drive(states[2].speedMetersPerSecond);
        BR.Drive(states[3].speedMetersPerSecond);
        

        if (Math.abs(chassisSpeeds.vxMetersPerSecond) + Math.abs(chassisSpeeds.vyMetersPerSecond) + Math.abs(chassisSpeeds.omegaRadiansPerSecond) < 1e-4)
            return;

        FL.SetTargetAngle(states[0].angle.getDegrees());
        FR.SetTargetAngle(states[1].angle.getDegrees());
        BL.SetTargetAngle(states[2].angle.getDegrees());
        BR.SetTargetAngle(states[3].angle.getDegrees());

    }

    void UpdateModules() {
        FL.Update(0.02);
        FR.Update(0.02);
        BL.Update(0.02);
        BR.Update(0.02);

        positions[0] = FL.GetPosition();
        positions[1] = FR.GetPosition();
        positions[2] = BL.GetPosition();
        positions[3] = BR.GetPosition();

        states[0] = FL.GetState();
        states[1] = FR.GetState();
        states[2] = BL.GetState();
        states[3] = BR.GetState();
    }


    void UpdateOdometry() {
        Rotation2d gyroAngle = GetGyroAngle();
        odometry.update(gyroAngle, positions);
        OdometryOutPose = odometry.getPoseMeters();
        OdometryOutPose = new Pose2d(
                new Translation2d(OdometryOutPose.getTranslation().getX(), OdometryOutPose.getTranslation().getY()),
                OdometryOutPose.getRotation());
        posePublisher.set(OdometryOutPose);
    }

    public Rotation2d GetGyroAngle() {
        return Rotation2d.fromDegrees(-gyro.getFusedHeading());
    }


    // DISPLAY

    void Display() {
        SmartDashboard.putString("Odom Localized", ("x: " + OdometryOutPose.getX() + ", y: " + OdometryOutPose.getY()));

        SmartDashboard.putNumber("Gyro angle", gyroAngle.getDegrees());
        odometryDisplay.setRobotPose(new Pose2d(OdometryOutPose.getX(), OdometryOutPose.getY(), gyroAngle));
    }


}
