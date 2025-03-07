// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ClimbConstants;
import frc.robot.autoalign.AutoHelper;
import frc.robot.autoalign.AutoHelper.CoralStation;
import frc.robot.autoalign.AutoHelper.ReefAlign;
import frc.robot.drivetrain.SwerveSubsystem;
import frc.robot.drivetrain.commands.SwerveJoystickDriveCommand;
import frc.robot.shooter.commands.TakeCoralCommand;
import frc.robot.shooter.elevator.ElevatorSubsystem;
import frc.robot.shooter.elevator.ElevatorTarget;
import frc.robot.shooter.elevator.ElevatorTargetSwitchSubsystem;
import frc.robot.shooter.shooter.ShooterSubsystem;
import frc.robot.shooter.shooter.ShooterSubsystem.ShooterMode;
import frc.robot.util.FixAllianceStartPose;

public class RobotContainer {
  private final CommandXboxController controller = new CommandXboxController(0);

  private final AutoHelper autoHelper;

  private final SwerveSubsystem swerveSubsystem;

  private final ElevatorSubsystem elevatorSubsystem;
  private final ElevatorTargetSwitchSubsystem elevatorTargetSwitchSubsystem;

  private final ShooterSubsystem shooterSubsystem;

  private final Supplier<Double> drivetrainSpeedCoeffSupplier;

  public ControlMode CM = ControlMode.CM_DEFAULT;

  public RobotContainer() {
    swerveSubsystem = new SwerveSubsystem();
    elevatorSubsystem = new ElevatorSubsystem();
    elevatorTargetSwitchSubsystem = new ElevatorTargetSwitchSubsystem();
    shooterSubsystem = new ShooterSubsystem();

    autoHelper = new AutoHelper(swerveSubsystem, elevatorSubsystem, shooterSubsystem);

    drivetrainSpeedCoeffSupplier = () -> CM == ControlMode.CM_CLIMB ? ClimbConstants.drivetrainSpeedCoeff : 1;

    configureBindings();
  }

  private void configureBindings() {

    swerveSubsystem.setDefaultCommand(new SwerveJoystickDriveCommand(
      swerveSubsystem,
      controller,
      () -> controller.getHID().getLeftX(),
      () -> controller.getHID().getLeftY(),
      () -> controller.getHID().getRightX(), 
      drivetrainSpeedCoeffSupplier,
      true
    ));

    configureIntakeBindings();
    configureReefBindings();
  }

  private void configureClimbBindings()
  {
    
  }

  private void configureIntakeBindings()
  {
    // Normal control mode
    //controller.leftTrigger().and(CM_IsDefault).whileTrue(autoHelper.AlignAndTakeCoral(CoralStation.Left));
    //controller.rightTrigger().and(CM_IsDefault).whileTrue(autoHelper.AlignAndTakeCoral(CoralStation.Right));
    controller.y().and(CM_IsDefault).whileTrue(Commands.defer(() -> new TakeCoralCommand(elevatorSubsystem, shooterSubsystem), Set.of()));

    //controller.x().toggleOnTrue(Commands.run(() -> shooterSubsystem.SetMode(ShooterMode.OUTTAKE)).finallyDo(() -> shooterSubsystem.SetMode(ShooterMode.IDLE)));


  }

  private void configureReefBindings()
  {
    //controller.x().and(CM_IsDefault).toggleOnTrue(Commands.defer(() -> autoHelper.AlignToClosestReefSide(ReefAlign.Left, 2), Set.of()));
    //controller.b().and(CM_IsDefault).toggleOnTrue(Commands.defer(() -> autoHelper.AlignToClosestReefSide(ReefAlign.Right, 2), Set.of()));

    controller.leftBumper().and(CM_IsDefault).onTrue(Commands.runOnce(() -> elevatorTargetSwitchSubsystem.DecreaseTarget()));
    controller.rightBumper().and(CM_IsDefault).onTrue(Commands.runOnce(() -> elevatorTargetSwitchSubsystem.IncreaseTarget()));


    Command ascendToReefCommand = Commands.defer(() -> elevatorSubsystem.GoToCommand(elevatorTargetSwitchSubsystem.GetTarget(true)).finallyDo((interrupted) -> {

      if(interrupted)
      elevatorSubsystem.SetTarget(ElevatorTarget.IDLE);

    }), Set.of());

    controller.a().and(CM_IsDefault).and(() -> elevatorTargetSwitchSubsystem.GetTarget(false) != null).toggleOnTrue(ascendToReefCommand);

    
    controller.a().and(CM_IsDefault)
    .and(() -> 
    elevatorSubsystem.AtTarget(elevatorSubsystem.GetTarget()) && 
    elevatorSubsystem.GetTarget().IsReefTarget() &&
    elevatorTargetSwitchSubsystem.GetTarget(false) == null)
     .whileTrue(shooterSubsystem.ShootCommand().finallyDo(() -> elevatorSubsystem.SetTarget(ElevatorTarget.IDLE)));
     
  }


  public Command getAutonomousCommand() {


    Pose2d pose = swerveSubsystem.GetRobotPose();

    Pose2d targetPose = new Pose2d(5.036, 4.025, Rotation2d.kZero);


      Pose2d allianceOrientedPose = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red ? FlippingUtil.flipFieldPose(targetPose) : targetPose;
      
      return Commands.runOnce(() -> System.out.println("AAAAAAAAAAAAAAAAAAAAUTO START"))
      .andThen(new FixAllianceStartPose(swerveSubsystem))
      .andThen(autoHelper.DriveToPose(allianceOrientedPose, false))
      .andThen(Commands.run(() -> shooterSubsystem.SetMode(ShooterMode.OUTTAKE)))
      .finallyDo(() -> shooterSubsystem.SetMode(ShooterMode.IDLE));
      
      
  }

  public void StopDrivetrain()
  {
    swerveSubsystem.Stop();
  }



  final BooleanSupplier CM_IsDefault = GetCMSupplier(ControlMode.CM_DEFAULT);
  final BooleanSupplier CM_IsReefAlign = GetCMSupplier(ControlMode.CM_REEFALIGN);
  final BooleanSupplier CM_IsClimb = GetCMSupplier(ControlMode.CM_CLIMB);

  BooleanSupplier GetCMSupplier(ControlMode cm)
  {
    return () -> CM == cm;
  }

  public enum ControlMode{
    CM_DEFAULT,
    CM_REEFALIGN,
    CM_CLIMB
  }
}
