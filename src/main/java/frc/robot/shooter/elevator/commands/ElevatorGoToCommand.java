package frc.robot.shooter.elevator.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.drivetrain.SwerveSubsystem;
import frc.robot.shooter.elevator.ElevatorSubsystem;
import frc.robot.shooter.elevator.ElevatorTarget;

public class ElevatorGoToCommand extends Command {
    
    ElevatorSubsystem _elevator;
    ElevatorTarget _target;

    public ElevatorGoToCommand(ElevatorSubsystem elevator, ElevatorTarget target) {
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
    }

    @Override
    public boolean isFinished() {
        return _elevator.AtTarget(_target);
    }
}
