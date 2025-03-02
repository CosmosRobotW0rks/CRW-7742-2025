package frc.robot.shooter.elevator.commands;

import edu.wpi.first.wpilibj2.command.Command;
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
        System.out.printf("ElevatorGoToCommand end: %d\n", interrupted ? 1 : 0);
    }

    @Override
    public boolean isFinished() {
        boolean atTarget = _elevator.AtTarget(_target);
        return atTarget;
    }
}
