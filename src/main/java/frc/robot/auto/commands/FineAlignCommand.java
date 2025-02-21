package frc.robot.auto.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.drivetrain.SwerveSubsystem;

public class FineAlignCommand extends Command {

    private final SwerveSubsystem swerve;

    private final PIDController[] controllers = new PIDController[]
    {
        new PIDController(AutoConstants.FineAlignDrive_kP, AutoConstants.FineAlignDrive_kI, AutoConstants.FineAlignDrive_kD),
        new PIDController(AutoConstants.FineAlignDrive_kP, AutoConstants.FineAlignDrive_kI, AutoConstants.FineAlignDrive_kD),
        new PIDController(AutoConstants.FineAlignAngle_kP, AutoConstants.FineAlignAngle_kI, AutoConstants.FineAlignAngle_kD)
    };

    public FineAlignCommand(SwerveSubsystem swerve, Pose2d targetPose)
    {
        this.swerve = swerve;
        addRequirements(swerve);
        
        double[] targetPoseDouble = Pose2DToDouble(targetPose);

        for(int i = 0; i < 3; i++)
        {
            controllers[i].setSetpoint(targetPoseDouble[i]);
        }

        controllers[0].setTolerance(AutoConstants.FineAlignTolerance_Translation);
        controllers[1].setTolerance(AutoConstants.FineAlignTolerance_Translation);
        controllers[2].setTolerance(AutoConstants.FineAlignTolerance_Rotation);


    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double[] outputs = new double[3];
        double[] currentPose = Pose2DToDouble(swerve.GetRobotPose());

        for(int i = 0; i < 3; i++)
        {
            outputs[i] = controllers[i].calculate(currentPose[i]);
        }

        ApplyOutputs(outputs);
    }


    @Override
    public boolean isFinished() {
        return controllers[0].atSetpoint() && controllers[1].atSetpoint() && controllers[2].atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.Stop();
    }

    private void ApplyOutputs(double[] outputs)
    {
        for(int i = 0; i < 2; i++)
        {
            outputs[i] = outputs[i] < -AutoConstants.FineAlignMaxDriveSpeed ? -AutoConstants.FineAlignMaxDriveSpeed : outputs[i];
            outputs[i] = outputs[i] > AutoConstants.FineAlignMaxRotSpeed ? AutoConstants.FineAlignMaxRotSpeed : outputs[i];
        }

        outputs[2] = outputs[2] < -AutoConstants.MaxRotSpeed ? -AutoConstants.MaxRotSpeed : outputs[2];
        outputs[2] = outputs[2] > AutoConstants.MaxRotSpeed ? AutoConstants.MaxRotSpeed : outputs[2];

        swerve.SetFieldOrientedChassisSpeeds(new ChassisSpeeds(outputs[0], outputs[1], outputs[2]));
    }


    private double[] Pose2DToDouble(Pose2d pose)
    {
        return new double[] { pose.getTranslation().getX(), pose.getTranslation().getY(), pose.getRotation().getRadians() };
    }



}
