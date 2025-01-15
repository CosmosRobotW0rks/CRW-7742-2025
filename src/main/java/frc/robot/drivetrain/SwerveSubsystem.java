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

    OldSwerveModule module = new OldSwerveModule(4, 6);


    double speed = 0;

    public SwerveSubsystem()
    {
        
    }

    public void SetAngle(double deg)
    {
        System.out.printf("Deg: %f\n", deg);
        module.SetTargetAngle(deg);
    }

    public void SetSpeed(double ms)
    {
        System.out.printf("Speed: %f\n", ms);
        module.SetSpeed(ms);
    }

    public void IncreaseSpeed(double delta)
    {
        speed += delta;
        SetSpeed(speed);
    }

    @Override
    public void periodic() {
        
        module.Update(0.02);
    }
}
