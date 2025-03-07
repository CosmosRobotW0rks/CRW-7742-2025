package frc.robot.util;

import java.util.Optional;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.drivetrain.SwerveSubsystem;
import frc.robot.util.Elastic;

public class FixAllianceStartPose extends Command {
    SwerveSubsystem swerveSubsystem;
    
    public FixAllianceStartPose(SwerveSubsystem swerve)
    {
        swerveSubsystem = swerve;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        
        Pose2d blueStartPose = new Pose2d(7.572, 4.025, Rotation2d.kZero);
        Pose2d redStartPose = FlippingUtil.flipFieldPose(blueStartPose);

        Optional<Alliance> alliance = DriverStation.getAlliance();
        boolean isRed = !alliance.isEmpty() && alliance.get() == Alliance.Red;

        swerveSubsystem.SetRobotPose(isRed ? redStartPose : blueStartPose);

    }

    @Override
    public boolean isFinished() {
        return true;

    }
}
