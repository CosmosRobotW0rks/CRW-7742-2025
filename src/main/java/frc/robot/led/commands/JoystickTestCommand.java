package frc.robot.led.commands;


import java.time.Period;
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

public class JoystickTestCommand extends Command {
    CommandXboxController joy;
    LEDSubsystem led;

    public JoystickTestCommand(LEDSubsystem led, CommandXboxController joy)
    {
        this.led = led;
        this.joy = joy;


        System.out.println("CONSTRUCTORRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR");


        addRequirements(led);
    }

    @Override
    public void initialize() {
        System.out.println("COMMAND INTINTITNI");
    }

    @Override
    public void execute() {

        System.out.println("CMD EXEC");
        led.Set(joy.getLeftY());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("COMMAND ENDDNENDENDE");
    }
}
