// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.drivetrain.SwerveSubsystem;
import frc.robot.drivetrain.commands.SwerveJoystickDriveCommand;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;

public class RobotContainer {
  private final CommandXboxController controller = new CommandXboxController(0);

  private final SwerveSubsystem swerve = new SwerveSubsystem();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    
    swerve.setDefaultCommand(new SwerveJoystickDriveCommand(
      swerve,
      () -> controller.getHID().getLeftX(),
      () -> controller.getHID().getLeftY(),
      () -> controller.getHID().getRightX(), 
      true
    ));

    
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
