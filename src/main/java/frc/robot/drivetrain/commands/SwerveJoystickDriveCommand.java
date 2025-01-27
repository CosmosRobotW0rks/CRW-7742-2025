package frc.robot.drivetrain.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.drivetrain.SwerveSubsystem;

public class SwerveJoystickDriveCommand extends Command {
    SwerveSubsystem swerve;

    final Supplier<Double> suppX, suppY, suppRot;
    final boolean deadzoneEnabled;

    SlewRateLimiter filterX = new SlewRateLimiter(DriveConstants.MaxDriveAccel);
    SlewRateLimiter filterY = new SlewRateLimiter(DriveConstants.MaxDriveAccel);
    SlewRateLimiter filterRot = new SlewRateLimiter(DriveConstants.MaxRotAccel);

    private double lastJoystickUpdateTimestamp = 0;

    public SwerveJoystickDriveCommand(SwerveSubsystem swerve, Supplier<Double> xspeed, Supplier<Double> yspeed, Supplier<Double> rotspeed, boolean deadzoneEnabled) {
        this.swerve = swerve;
        this.deadzoneEnabled = deadzoneEnabled;

        this.suppX = xspeed;
        this.suppY = yspeed;
        this.suppRot = rotspeed;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
    }

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
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println(String.format("Swerve Joystick Drive Command ended (Interrupted: %d)", interrupted));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    void JoystickDrive(double x, double y, double rot) {
        x =     x < -1 ? -1 : x   > 1 ? 1 : x;
        y =     y < -1 ? -1 : y   > 1 ? 1 : y;
        rot = rot < -1 ? -1 : rot > 1 ? 1 : rot;

        
        double targetXspeed = -y * DriveConstants.MaxDriveSpeed;
        double targetYspeed = -x * DriveConstants.MaxDriveSpeed;
        double targetRotSpeed = rot * DriveConstants.MaxRotSpeed;

        targetXspeed = filterX.calculate(targetXspeed);
        targetYspeed = filterY.calculate(targetYspeed);
        targetRotSpeed = filterRot.calculate(targetRotSpeed);


        /*
        
        double now = Timer.getFPGATimestamp();
        double delta = now - lastJoystickUpdateTimestamp;
        lastJoystickUpdateTimestamp = now;

        ChassisSpeeds chassisSpeeds = swerve.GetChassisSpeeds();

        targetXspeed = GetAccelLimitedSpeed(chassisSpeeds.vxMetersPerSecond, targetXspeed, delta, DriveConstants.MaxDriveAccel);
        targetYspeed = GetAccelLimitedSpeed(chassisSpeeds.vyMetersPerSecond, targetYspeed, delta, DriveConstants.MaxDriveAccel);
        targetRotspeed = GetAccelLimitedSpeed(chassisSpeeds.omegaRadiansPerSecond, targetRotspeed, delta, DriveConstants.MaxRotAccel);
*/
        swerve.SetFieldOrientedChassisSpeeds(targetXspeed, targetYspeed, targetRotSpeed);
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
