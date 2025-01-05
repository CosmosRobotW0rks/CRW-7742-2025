package frc.robot.drivetrain;

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

    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(WIDTH, HEIGHT),
            new Translation2d(WIDTH, -HEIGHT),
            new Translation2d(-WIDTH, HEIGHT),
            new Translation2d(-WIDTH, -HEIGHT));

    public AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    Field2d odometryDisplay = new Field2d();

    public boolean Enabled = true;

    boolean homed = false;

    double x_spd = 0;
    double y_spd = 0;
    double th_spd = 0;

    double old_time;

    Translation3d drivetr = new Translation3d();

    // -- ODOMETRY SHIT --

    public Rotation2d gyroAngle = new Rotation2d();
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    SwerveModuleState[] states = new SwerveModuleState[4];
    SwerveDriveOdometry odometry;
    public Pose2d OdometryOutPose;

    public Transform2d OdometryOffset = new Transform2d();

    // -- END OF ODOMETRY SHIT--

    public SwerveSubsystem() {
        UpdateModules();
        odometry = new SwerveDriveOdometry(kinematics, gyroAngle, positions);

        odometryDisplay.setRobotPose(new Pose2d(new Translation2d(), new Rotation2d()));
        SmartDashboard.putData("field", odometryDisplay);
        //SmartDashboard.putData("Home", new SwerveSetDegrees(this, 0, 0, 0, 0, true));
        //SmartDashboard.putData("Cross", new SwerveSetDegrees(this, 45, -45, -45, 45, true));
    }

    public void SetTargetAbsolute(double fl, double fr, double bl, double br, boolean forceForward) {
        FL.SetTargetAngle(fl, forceForward);
        FR.SetTargetAngle(fr, forceForward);
        BL.SetTargetAngle(bl, forceForward);
        BR.SetTargetAngle(br, forceForward);

        if (fl == 0 && fr == 0 && bl == 0 && br == 0)
            homed = true;
        else
            homed = false;

        // TODO: make sure homed can be safely set to false here

    }

    public void SetTargetAbsolute(double tl, double tr, double bl, double br) {
        SetTargetAbsolute(tl, tr, bl, br, false);
    }

    public void SetDriveTR(Translation3d t3d) {
        drivetr = t3d;
    }

    void Drive() {
        double new_time = Timer.getFPGATimestamp();

        SetXYSpeeds(drivetr, new_time - old_time);

        old_time = new_time;

        ChassisSpeeds fieldOrientedXYSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                -x_spd,
                -y_spd,
                -drivetr.getZ(),
                gyroAngle); // Gyro is upside down?

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(fieldOrientedXYSpeeds);

        FL.Drive(states[0].speedMetersPerSecond);
        FR.Drive(states[1].speedMetersPerSecond);
        BL.Drive(states[2].speedMetersPerSecond);
        BR.Drive(states[3].speedMetersPerSecond);

        if (Math.abs(fieldOrientedXYSpeeds.vxMetersPerSecond) + Math.abs(fieldOrientedXYSpeeds.vyMetersPerSecond)
                + Math.abs(fieldOrientedXYSpeeds.omegaRadiansPerSecond) < 1e-4)
            return;

        FL.SetTargetAngle(states[0].angle.getDegrees());
        FR.SetTargetAngle(states[1].angle.getDegrees());
        BL.SetTargetAngle(states[2].angle.getDegrees());
        BR.SetTargetAngle(states[3].angle.getDegrees());
    }

    void SetXYSpeeds(Translation3d t3d, double delta) {

        final double max_x_accel = DriveConstants.MaxAccel_X;
        final double max_y_accel = DriveConstants.MaxAccel_Y;

        if (x_spd < t3d.getX()) {
            x_spd += delta * max_x_accel;

            if (x_spd > t3d.getX())
                x_spd = t3d.getX();
        }

        if (x_spd > t3d.getX()) {
            x_spd -= delta * max_x_accel;

            if (x_spd < t3d.getX())
                x_spd = t3d.getX();
        }

        if (y_spd < t3d.getY()) {
            y_spd += delta * max_y_accel;

            if (y_spd > t3d.getY())
                y_spd = t3d.getY();
        }

        if (y_spd > t3d.getY()) {
            y_spd -= delta * max_y_accel;

            if (y_spd < t3d.getY())
                y_spd = t3d.getY();
        }
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
        odometryDisplay.setRobotPose(new Pose2d(OdometryOutPose.getX(), OdometryOutPose.getY() + 5.548, gyroAngle));
    }

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
