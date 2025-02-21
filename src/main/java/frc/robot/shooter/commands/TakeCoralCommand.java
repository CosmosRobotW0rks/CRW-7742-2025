package frc.robot.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autoalign.AutoHelper.CoralStation;
import frc.robot.shooter.elevator.ElevatorSubsystem;
import frc.robot.shooter.elevator.ElevatorTarget;
import frc.robot.util.Elastic;

public class TakeCoralCommand extends Command {
    ElevatorSubsystem elevator;

    Status status = Status.WaitingForElevator;


    public TakeCoralCommand(ElevatorSubsystem elevator) {
        this.elevator = elevator;


        addRequirements(elevator);

    }
    
    @Override
    public void initialize() {

        elevator.SetTarget(ElevatorTarget.CORALSTAT);
        this.status = Status.WaitingForElevator;
    }

    @Override
    public void execute() {
        
        if(status == Status.WaitingForElevator) {

            if(elevator.GetTarget() != ElevatorTarget.CORALSTAT)
            {
                Elastic.SendError("Aborting command","Elevator target is not CORALSTAT");
                this.cancel();
                return;
            }

            if(elevator.AtTarget(ElevatorTarget.CORALSTAT)) status = Status.WaitingForIntake;
            else return;
        }

        if(true /* Got the coral */)
        {
            status = Status.Done;
        }
    }

    @Override
    public boolean isFinished() {
        return status == Status.Done;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.SetTarget(ElevatorTarget.IDLE);
    }


    enum Status
    {
        WaitingForElevator,
        WaitingForIntake,
        Done
    }
}
