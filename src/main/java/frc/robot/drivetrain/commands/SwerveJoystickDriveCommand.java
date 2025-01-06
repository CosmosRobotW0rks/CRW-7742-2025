package frc.robot.drivetrain.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.drivetrain.SwerveSubsystem;

public class SwerveJoystickDriveCommand extends Command {
    SwerveSubsystem swerve;

    final Supplier<Double> speedFuncX, speedFuncY, speedFuncRot;

    final boolean deadzoneEnabled;

    public SwerveJoystickDriveCommand(SwerveSubsystem swerve, Supplier<Double> xspeed, Supplier<Double> yspeed, Supplier<Double> rotspeed, boolean deadzoneEnabled) {
        this.swerve = swerve;
        this.deadzoneEnabled = deadzoneEnabled;

        this.speedFuncX = xspeed;
        this.speedFuncY = yspeed;
        this.speedFuncRot = rotspeed;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double xspeed = speedFuncX.get();
        double yspeed = speedFuncY.get();
        double rotspeed = speedFuncRot.get();

        if (deadzoneEnabled) {
            final double dz_x = DriveConstants.JOYDeadzone_X;
            final double dz_y = DriveConstants.JOYDeadzone_Y;
            final double dz_rot = DriveConstants.JOYDeadzone_Rot;

            xspeed = ApplyDeadzone(xspeed, dz_x);
            yspeed = ApplyDeadzone(yspeed, dz_y);
            rotspeed = ApplyDeadzone(rotspeed, dz_rot);
        }
        Translation3d t3d = new Translation3d(xspeed, yspeed, rotspeed);

        // TODO: Make it move faster than 1 m/s
        System.out.printf("X: %f / Y: %f / Rot: %f\n", xspeed, yspeed, rotspeed);
        swerve.FromJoystick(xspeed, yspeed, rotspeed);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println(String.format("Swerve Joystick Drive Command ended (Interrupted: %d)", interrupted));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // ----------------

    double ApplyDeadzone(double val, double deadzone) {
        val = val > 0 && val <= deadzone ? 0 : val;
        val = val < 0 && val >= -deadzone ? 0 : val;

        return val;
    }
}
