package frc.robot.shooter.shooter;
import java.util.Dictionary;
import java.util.Hashtable;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.shooter.elevator.commands.ElevatorGoToCommand;


public class ShooterSubsystem extends SubsystemBase  {

    private SparkMax spark;
    private RelativeEncoder encoder;
    private SparkClosedLoopController controller;

    private ShooterMode mode = ShooterMode.IDLE;
    private boolean IsIntakeComplete = false;
    private boolean IsOuttakeComplete = false;

    public ShooterSubsystem()
    {
        // MOTOR CONFIG
        spark = new SparkMax(ShooterConstants.CANID, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kBrake);

        config.closedLoop.pidf(ShooterConstants.PIDF_P, ShooterConstants.PIDF_I, ShooterConstants.PIDF_D, ShooterConstants.PIDF_F);

        spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = spark.getEncoder();
        controller = spark.getClosedLoopController();
    }


    private void SetMode(ShooterMode mode)
    {
        this.mode = mode;
        
        if(mode == ShooterMode.INTAKE) IsIntakeComplete = false;
        if(mode == ShooterMode.OUTTAKE) IsOuttakeComplete = false;
    }

    public Command IntakeCommand()
    {
        return new Command() {
            
            @Override
            public void initialize() {
                SetMode(ShooterMode.INTAKE);
            }

            @Override
            public boolean isFinished() {
                return IsIntakeComplete;
            }

            @Override
            public void end(boolean interrupted) {
                SetMode(ShooterMode.IDLE);
            }
        };
    }

    public Command ShootCommand()
    {
        return new Command() {
            
            @Override
            public void initialize() {
                SetMode(ShooterMode.OUTTAKE);
            }

            @Override
            public boolean isFinished() {
                return IsOuttakeComplete;
            }

            @Override
            public void end(boolean interrupted) {
                SetMode(ShooterMode.IDLE);
            }
        };
    }

    @Override
    public void periodic() {
        switch(mode)
        {
            case IDLE:
                controller.setReference(0, ControlType.kVelocity);
                break;
            case INTAKE:
                IntakePeriodic();
                break;
            case OUTTAKE:
                OuttakePeriodic();
                break;
        }
    }

    private void IntakePeriodic()
    {
        controller.setReference(ShooterConstants.IntakeVelocity, ControlType.kVelocity);

        if(encoder.getVelocity() < ShooterConstants.IntakeThresholdVelocity)
        {
            IsIntakeComplete = true;
            SetMode(ShooterMode.IDLE);
        }
    }

    private void OuttakePeriodic()
    {
        controller.setReference(ShooterConstants.OuttakeVelocity, ControlType.kVelocity);

        if(encoder.getVelocity() > ShooterConstants.OuttakeThresholdVelocity)
        {
            IsOuttakeComplete = true;
            SetMode(ShooterMode.IDLE);
        }
    }


    public enum ShooterMode
    {
        IDLE,
        INTAKE,
        OUTTAKE
    }
}
