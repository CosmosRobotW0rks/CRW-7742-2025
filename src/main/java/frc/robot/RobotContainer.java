// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Random;
import java.util.Set;
import java.util.function.Supplier;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autoalign.AutoHelper;
import frc.robot.autoalign.AutoHelper.CoralStation;
import frc.robot.autoalign.AutoHelper.ReefAlign;
import frc.robot.drivetrain.SwerveSubsystem;
import frc.robot.drivetrain.commands.SwerveJoystickDriveCommand;
import frc.robot.shooter.commands.TakeCoralCommand;
import frc.robot.shooter.elevator.ElevatorSubsystem;
import frc.robot.shooter.elevator.ElevatorTarget;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;

public class RobotContainer {
  private final CommandXboxController controller = new CommandXboxController(0);

  private final AutoHelper autoHelper;

  private final SwerveSubsystem swerveSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;

  public RobotContainer() {
    swerveSubsystem = new SwerveSubsystem();
    elevatorSubsystem = new ElevatorSubsystem();

    autoHelper = new AutoHelper(swerveSubsystem, elevatorSubsystem);

    configureBindings();
  }

  private void configureBindings() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickDriveCommand(
      swerveSubsystem,
      controller,
      () -> controller.getHID().getLeftX(),
      () -> controller.getHID().getLeftY(),
      () -> controller.getHID().getRightX(), 
      true
    ));
    configureIntakeBindings();
    
    controller.x().whileTrue(Commands.defer(() -> autoHelper.AlignToClosestReefSide(ReefAlign.Left, 2), Set.of()));
    controller.y().whileTrue(Commands.defer(() -> autoHelper.AlignToClosestReefSide(ReefAlign.Mid, 2), Set.of()));
    controller.b().whileTrue(Commands.defer(() -> autoHelper.AlignToClosestReefSide(ReefAlign.Right, 2), Set.of()));

    //controller.a().onChange(Commands.defer(() -> elevatorSubsystem.GoTo(elevatorSubsystem.GetTarget() == ElevatorTarget.IDLE ? ElevatorTarget.L4 : ElevatorTarget.IDLE), Set.of()));
  }

  private void configureIntakeBindings()
  {
    controller.leftTrigger().whileTrue(autoHelper.AlignAndTakeCoral(CoralStation.Left));
    controller.rightTrigger().whileTrue(autoHelper.AlignAndTakeCoral(CoralStation.Right));


    /*
    controller.leftTrigger().whileTrue(Commands.defer(() -> 

    autoHelper.AlignToCoralStation(CoralStation.Left)
    .andThen(elevatorSubsystem.GoToCommand(ElevatorTarget.CORALSTAT).finallyDo(() -> elevatorSubsystem.SetTarget(ElevatorTarget.IDLE))) // TODO: remove flicker
    .andThen(elevatorSubsystem.SetTargetCommand(ElevatorTarget.IDLE))


    , Set.of()));
    controller.rightTrigger().whileTrue(Commands.defer(() -> 

    autoHelper.AlignToCoralStation(CoralStation.Right)
    .andThen(elevatorSubsystem.GoToCommand(ElevatorTarget.CORALSTAT))
    .andThen(elevatorSubsystem.SetTargetCommand(ElevatorTarget.IDLE))

    , Set.of()));

    */
  }


  public Command getAutonomousCommand() {
    return (new PathPlannerAuto("TESTAUTO")).andThen(swerveSubsystem.StopCommand());
  }

  public void StopDrivetrain()
  {
    swerveSubsystem.Stop();
  }
}
