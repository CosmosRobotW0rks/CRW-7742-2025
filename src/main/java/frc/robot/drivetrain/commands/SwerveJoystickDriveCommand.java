package frc.robot.drivetrain.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.drivetrain.SwerveSubsystem;
import frc.robot.led.LEDSubsystem;

public class SwerveJoystickDriveCommand extends Command {
    SwerveSubsystem swerve;

    CommandXboxController _j;

    final Supplier<Double> suppX, suppY, suppRot;
    final boolean deadzoneEnabled;

    SlewRateLimiter filterX = new SlewRateLimiter(DriveConstants.MaxDriveAccel);
    SlewRateLimiter filterY = new SlewRateLimiter(DriveConstants.MaxDriveAccel);
    SlewRateLimiter filterRot = new SlewRateLimiter(DriveConstants.MaxRotAccel);

    public SwerveJoystickDriveCommand(SwerveSubsystem swerve, CommandXboxController j, Supplier<Double> xspeed, Supplier<Double> yspeed, Supplier<Double> rotspeed, boolean deadzoneEnabled) {
        this.swerve = swerve;
        this.deadzoneEnabled = deadzoneEnabled;

        this.suppX = xspeed;
        this.suppY = yspeed;
        this.suppRot = rotspeed;

        _j = j;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {

        start = swerve.GetRobotPose().getTranslation();
    }


    Translation2d start;

    @Override
    public void execute() {
        double xpercent = suppX.get();
        double ypercent = suppY.get();
        double rotpercent = suppRot.get();
/*
        xpercent = Math.pow(xpercent, 2) * Math.signum(xpercent);
        ypercent = Math.pow(ypercent, 2) * Math.signum(ypercent);
        rotpercent = Math.pow(rotpercent, 2) * Math.signum(rotpercent);
*/

        if (deadzoneEnabled) {
            xpercent = ApplyDeadzone(xpercent, DriveConstants.JOYDeadzone_X);
            ypercent = ApplyDeadzone(ypercent, DriveConstants.JOYDeadzone_Y);
            rotpercent = ApplyDeadzone(rotpercent, DriveConstants.JOYDeadzone_Rot);
        }

        JoystickDrive(xpercent, ypercent, rotpercent);


        Translation2d res = start.minus(swerve.GetRobotPose().getTranslation());

        SmartDashboard.putNumberArray("MOVETR", new double[] {res.getX(), res.getY()});
        
        if(_j.getHID().getYButtonPressed())
        {
            start = swerve.GetRobotPose().getTranslation();
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println(String.format("Swerve Joystick Drive Command ended (Interrupted: %d)", interrupted ? 1 : 0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    boolean xZero = true;
    boolean yZero = true;
    boolean rotZero = true;
    void JoystickDrive(double x, double y, double rot) {
        x =     x < -1 ? -1 : x   > 1 ? 1 : x;
        y =     y < -1 ? -1 : y   > 1 ? 1 : y;
        rot = rot < -1 ? -1 : rot > 1 ? 1 : rot;

        
        double targetXspeed = -y * DriveConstants.MaxDriveSpeed;
        double targetYspeed = -x * DriveConstants.MaxDriveSpeed;
        double targetRotSpeed = -rot * DriveConstants.MaxRotSpeed;

        targetXspeed = filterX.calculate(targetXspeed);
        targetYspeed = filterY.calculate(targetYspeed);
        targetRotSpeed = filterRot.calculate(targetRotSpeed);


        if(xZero && yZero && rotZero && targetXspeed == 0 && targetYspeed == 0 && targetRotSpeed == 0) return;


        xZero = targetXspeed == 0;
        yZero = targetYspeed == 0;
        rotZero = targetRotSpeed == 0;

        ChassisSpeeds cs = new ChassisSpeeds(targetXspeed, targetYspeed, targetRotSpeed);

        swerve.SetFieldOrientedChassisSpeeds(cs);
    }

    
    double ApplyDeadzone(double val, double deadzone) {
        val = val > 0 && val <= deadzone ? 0 : val;
        val = val < 0 && val >= -deadzone ? 0 : val;

        return val;
    }

    double GetAccelLimitedSpeed(double previousSpeed, double targetSpeed, double deltaTime, double maxAccel)
    {
        double speedDiff = targetSpeed - previousSpeed;
        double accel = speedDiff / deltaTime;

        if (Math.abs(accel) > maxAccel)
        {
            accel = accel > 0 ? maxAccel : -maxAccel;
        }

        return previousSpeed + (accel * deltaTime);
    }
}
