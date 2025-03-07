package frc.robot.shooter.shooter.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.shooter.elevator.ElevatorSubsystem;
import frc.robot.shooter.elevator.ElevatorTarget;
import frc.robot.shooter.shooter.ShooterSubsystem;
import frc.robot.shooter.shooter.ShooterSubsystem.ShooterMode;
import frc.robot.util.Elastic;

public class ShooterJoystickCommand extends Command {
    
    public ShooterJoystickCommand(ShooterSubsystem shooter, Supplier<Double> outtakePercentage, Supplier<Double> intakePercentage)
    {
           
    }
}
