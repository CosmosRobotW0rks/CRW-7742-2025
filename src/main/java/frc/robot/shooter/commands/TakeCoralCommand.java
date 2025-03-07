package frc.robot.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.shooter.elevator.ElevatorSubsystem;
import frc.robot.shooter.elevator.ElevatorTarget;
import frc.robot.shooter.shooter.ShooterSubsystem;
import frc.robot.shooter.shooter.ShooterSubsystem.ShooterMode;
import frc.robot.util.Elastic;

public class TakeCoralCommand extends Command {
    ElevatorSubsystem elevator;

    ShooterSubsystem shooter;

    Status status = Status.WaitingForElevator;


    public TakeCoralCommand(ElevatorSubsystem elevator, ShooterSubsystem shooter) {
        this.elevator = elevator;
        this.shooter = shooter;


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

            if(elevator.AtTarget(ElevatorTarget.CORALSTAT))
            {
                shooter.SetMode(ShooterMode.INTAKE);
                status = Status.WaitingForIntake;
            }
            else return;
        }

        if(shooter.IntakeCompleted())
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
        shooter.SetMode(ShooterMode.IDLE);
    }


    enum Status
    {
        WaitingForElevator,
        WaitingForIntake,
        Done
    }
}
