package frc.robot.autoalign.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.autoalign.AutoHelper;
import frc.robot.drivetrain.SwerveSubsystem;

public class ExecuteWhenNearPosition extends Command {
    private final AutoHelper autoHelper;
    private final Runnable action;
    private final Translation2d pos;
    private final double tolerance;
    private final boolean runOnce;


    private boolean ran = false;


    public ExecuteWhenNearPosition(AutoHelper autoHelper, Translation2d pos, double tolerance, Runnable action, boolean runOnce)
    {
        this.autoHelper = autoHelper;
        this.pos = pos;
        this.tolerance = tolerance;
        this.action = action;
        this.runOnce = runOnce;
    }

    public ExecuteWhenNearPosition(AutoHelper autoHelper, Translation2d pos, double tolerance, boolean runOnce)
    {
        this.autoHelper = autoHelper;
        this.pos = pos;
        this.tolerance = tolerance;
        this.action = null;
        this.runOnce = runOnce;
    }

    @Override
    public void initialize() {
        System.out.println("ExecuteWhenNearPosition initialized");
    }

    @Override
    public void execute() {
        if(autoHelper.NearFieldPosition(pos, tolerance))
        {
            ran = true;

            if(action != null)
            action.run();
        }
    }


    @Override
    public boolean isFinished() {
        return runOnce && ran;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.printf("ExecuteWhenNearPosition ended, interrupted: %b\n", interrupted);
    }



}
