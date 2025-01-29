package frc.robot.elevator.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.drivetrain.SwerveSubsystem;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.elevator.ElevatorTarget;

public class ElevatorSetTargetCommand extends Command {
    
    ElevatorSubsystem _elevator;
    ElevatorTarget _target;

    public ElevatorSetTargetCommand(ElevatorSubsystem elevator, ElevatorTarget target) {
        this._elevator = elevator;
        this._target = target;
        addRequirements(_elevator);
    }

    @Override
    public void initialize() {
        _elevator.SetTarget(_target);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println(String.format("Swerve Joystick Drive Command ended (Interrupted: %d)", interrupted));
    }

    @Override
    public boolean isFinished() {
        return _elevator.AtTarget(_target);
    }
}
