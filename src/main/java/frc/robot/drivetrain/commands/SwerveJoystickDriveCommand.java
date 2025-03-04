package frc.robot.drivetrain.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.drivetrain.SwerveSubsystem;

public class SwerveJoystickDriveCommand extends Command {
    SwerveSubsystem swerve;

    CommandXboxController _j;

    final Supplier<Double> suppX, suppY, suppRot;
    final Supplier<Double> suppCoeff;

    final boolean deadzoneEnabled;

    SlewRateLimiter filterX = new SlewRateLimiter(DriveConstants.MaxDriveAccel);
    SlewRateLimiter filterY = new SlewRateLimiter(DriveConstants.MaxDriveAccel);
    SlewRateLimiter filterRot = new SlewRateLimiter(DriveConstants.MaxRotAccel);


    SlewRateLimiter[] accelFilters = new SlewRateLimiter[]{
        new SlewRateLimiter(DriveConstants.MaxDriveAccel),
        new SlewRateLimiter(DriveConstants.MaxDriveAccel),
        new SlewRateLimiter(DriveConstants.MaxRotAccel)
    };

    SlewRateLimiter[] deccelFilters = new SlewRateLimiter[]{
        new SlewRateLimiter(DriveConstants.MaxDriveDeccel),
        new SlewRateLimiter(DriveConstants.MaxDriveDeccel),
        new SlewRateLimiter(DriveConstants.MaxRotDeccel)
    };

    public SwerveJoystickDriveCommand(SwerveSubsystem swerve, CommandXboxController j, Supplier<Double> xspeed, Supplier<Double> yspeed, Supplier<Double> rotspeed, Supplier<Double> speedCoeff, boolean deadzoneEnabled) {
        this.swerve = swerve;
        this.deadzoneEnabled = deadzoneEnabled;

        this.suppX = xspeed;
        this.suppY = yspeed;
        this.suppRot = rotspeed;
        this.suppCoeff = speedCoeff;

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

        if (deadzoneEnabled) {
            xpercent = ApplyDeadzone(xpercent, DriveConstants.JOYDeadzone_X);
            ypercent = ApplyDeadzone(ypercent, DriveConstants.JOYDeadzone_Y);
            rotpercent = ApplyDeadzone(rotpercent, DriveConstants.JOYDeadzone_Rot);
        }

        xpercent = Math.pow(xpercent, 3);// * Math.signum(xpercent);
        ypercent = Math.pow(ypercent, 3);// * Math.signum(ypercent);
        //rotpercent = Math.pow(rotpercent, 3);// * Math.signum(rotpercent);



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


        double[] targetSpeeds = new double[]
        {
            -y * DriveConstants.MaxDriveSpeed * suppCoeff.get(),
            -x * DriveConstants.MaxDriveSpeed * suppCoeff.get(),
            -rot * DriveConstants.MaxRotSpeed * suppCoeff.get()
        };


        ChassisSpeeds frcs = swerve.GetRobotRelativeChassisSpeeds();
        
        ApplyFilter(targetSpeeds, new double[] {frcs.vxMetersPerSecond, frcs.vyMetersPerSecond, frcs.omegaRadiansPerSecond});


        if(xZero && yZero && rotZero && targetSpeeds[0] == 0 && targetSpeeds[1] == 0 && targetSpeeds[2] == 0) return;


        xZero = targetSpeeds[0] == 0;
        yZero = targetSpeeds[1] == 0;
        rotZero = targetSpeeds[2] == 0;

        ChassisSpeeds cs = new ChassisSpeeds(targetSpeeds[0],targetSpeeds[1],targetSpeeds[2]);

        swerve.SetFieldOrientedChassisSpeeds(cs);

    }

    void ApplyFilter(double[] targetSpeeds, double[] prevSpeeds)
    {
        for(int i = 0; i<3; i++)
        {
            double incSpeed = accelFilters[i].calculate(targetSpeeds[i]);
            double descSpeed = deccelFilters[i].calculate(targetSpeeds[i]);

            if(prevSpeeds[i] < targetSpeeds[i]) targetSpeeds[i] = incSpeed;
            else targetSpeeds[i] = descSpeed;
        }
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
