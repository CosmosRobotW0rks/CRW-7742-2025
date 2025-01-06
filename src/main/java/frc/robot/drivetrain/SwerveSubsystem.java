package frc.robot.drivetrain;

import static edu.wpi.first.units.Units.Inches;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase{

    final double WIDTH = 11.2; // Inches? TODO Add dimensions
    final double HEIGHT = 10.5;

    SwerveModule FL = new SwerveModule(Constants.SwerveConstants.AngleCANID_FL, Constants.SwerveConstants.DriveCANID_FL);
    SwerveModule FR = new SwerveModule(Constants.SwerveConstants.AngleCANID_FR, Constants.SwerveConstants.DriveCANID_FR);
    SwerveModule BL = new SwerveModule(Constants.SwerveConstants.AngleCANID_BL, Constants.SwerveConstants.DriveCANID_BL);
    SwerveModule BR = new SwerveModule(Constants.SwerveConstants.AngleCANID_BR, Constants.SwerveConstants.DriveCANID_BR);

    public AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    Field2d odometryDisplay = new Field2d();

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);

    public SwerveSubsystem() {

        UpdateModules();

        odometry = new SwerveDriveOdometry(kinematics, gyroAngle, positions);
        odometryDisplay.setRobotPose(new Pose2d(new Translation2d(), new Rotation2d()));

        SmartDashboard.putData("Field", odometryDisplay);

        //SmartDashboard.putData("Home", new SwerveSetDegrees(this, 0, 0, 0, 0, true));
        //SmartDashboard.putData("Cross", new SwerveSetDegrees(this, 45, -45, -45, 45, true));
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

    double lastJoystickUpdateTimestamp = 0;

    public void FromJoystick(double x, double y, double rot) {
        x =     x < -1 ? -1 : x   > 1 ? 1 : x;
        y =     y < -1 ? -1 : y   > 1 ? 1 : y;
        rot = rot < -1 ? -1 : rot > 1 ? 1 : rot;

        
        double targetXspeed = x * DriveConstants.MaxDriveSpeed;
        double targetYspeed = y * DriveConstants.MaxDriveSpeed;
        double targetRotspeed = rot * DriveConstants.MaxRotSpeed;
        
        double now = Timer.getFPGATimestamp();
        double delta = now - lastJoystickUpdateTimestamp;
        lastJoystickUpdateTimestamp = now;

        targetXspeed = GetAccelLimitedSpeed(-chassisSpeeds.vxMetersPerSecond, targetXspeed, delta, DriveConstants.MaxDriveAccel);
        targetYspeed = GetAccelLimitedSpeed(-chassisSpeeds.vyMetersPerSecond, targetYspeed, delta, DriveConstants.MaxDriveAccel);
        targetRotspeed = GetAccelLimitedSpeed(-chassisSpeeds.omegaRadiansPerSecond, targetRotspeed, delta, DriveConstants.MaxRotAccel);

        ChassisSpeeds fieldOrientedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                -targetXspeed,
                -targetYspeed,
                -targetRotspeed,
                gyroAngle);


        SetChassisSpeeds(fieldOrientedSpeeds);
    }

    public void SetChassisSpeeds(ChassisSpeeds cs)
    {
        chassisSpeeds = cs;
    }

    private double GetAccelLimitedSpeed(double previousSpeed, double targetSpeed, double deltaTime, double maxAccel)
    {
        double speedDiff = targetSpeed - previousSpeed;
        double accel = speedDiff / deltaTime;

        if (Math.abs(accel) > maxAccel)
        {
            accel = accel > 0 ? maxAccel : -maxAccel;
        }

        return previousSpeed + (accel * deltaTime);
    }


    // EOD
    
    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(WIDTH, HEIGHT),
            new Translation2d(WIDTH, -HEIGHT),
            new Translation2d(-WIDTH, HEIGHT),
            new Translation2d(-WIDTH, -HEIGHT));


    void Drive() {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);

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

    void Display() {
        SmartDashboard.putString("Odom Localized", ("x: " + OdometryOutPose.getX() + ", y: " + OdometryOutPose.getY()));

        SmartDashboard.putNumber("Gyro angle", gyroAngle.getDegrees());
        odometryDisplay.setRobotPose(new Pose2d(OdometryOutPose.getX(), OdometryOutPose.getY(), gyroAngle));
    }

    public Rotation2d gyroAngle = new Rotation2d();
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    SwerveModuleState[] states = new SwerveModuleState[4];
    SwerveDriveOdometry odometry;
    public Pose2d OdometryOutPose;

    void Odometry() {
        gyroAngle = Rotation2d.fromDegrees(-gyro.getFusedHeading());
        odometry.update(gyroAngle, positions);
        OdometryOutPose = odometry.getPoseMeters();
        OdometryOutPose = new Pose2d(
                new Translation2d(OdometryOutPose.getTranslation().getX(), OdometryOutPose.getTranslation().getY()),
                OdometryOutPose.getRotation());
    }

    public void SetOdometryPose(Pose2d pose) {
        odometry.resetPosition(gyroAngle, positions, pose);
    }

    @Override
    public void periodic() {
        UpdateModules();
        Odometry();
        Drive();
        Display();
    }
}
