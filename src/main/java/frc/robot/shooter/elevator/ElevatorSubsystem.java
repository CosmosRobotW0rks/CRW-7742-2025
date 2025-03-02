package frc.robot.shooter.elevator;
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
import frc.robot.shooter.elevator.commands.ElevatorGoToCommand;


public class ElevatorSubsystem extends SubsystemBase  {

    private SparkMax spark;
    private RelativeEncoder encoder;
    private SparkClosedLoopController controller;

    Dictionary<ElevatorTarget, Double> targetMap = new Hashtable<>();

    public ElevatorSubsystem()
    {
        // MOTOR CONFIG
        spark = new SparkMax(ElevatorConstants.CANID, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kBrake);

        config.closedLoop.pid(ElevatorConstants.PID_P, ElevatorConstants.PID_I, ElevatorConstants.PID_D);
        config.closedLoop.maxMotion.maxAcceleration(ElevatorConstants.MaxAccel);
        config.closedLoop.maxMotion.maxVelocity(ElevatorConstants.MaxVelocity);

        spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = spark.getEncoder();
        controller = spark.getClosedLoopController();

        // TARGET CONFIG
        targetMap.put(ElevatorTarget.IDLE, 0.0);
        targetMap.put(ElevatorTarget.CORALSTAT, ElevatorConstants.TARGET_CORALSTAT);
        targetMap.put(ElevatorTarget.L1, ElevatorConstants.TARGET_REEFL1);
        targetMap.put(ElevatorTarget.L2, ElevatorConstants.TARGET_REEFL2);
        targetMap.put(ElevatorTarget.L3, ElevatorConstants.TARGET_REEFL3);
        targetMap.put(ElevatorTarget.L4, ElevatorConstants.TARGET_REEFL4);


        // SMARTDASHBOARD
        SmartDashboard.putString("Elevator Target", target.name());
    }

    ElevatorTarget target = ElevatorTarget.IDLE;

    double targetSetTimestamp = 0;


    public ElevatorTarget GetTarget()
    {
        return target;
    }

    public void SetTarget(ElevatorTarget elevatorTarget)
    {
        target = elevatorTarget;
        targetSetTimestamp = Timer.getFPGATimestamp();

        SetTarget(targetMap.get(elevatorTarget));
    }

    public boolean AtTarget(ElevatorTarget target)
    {
        if(Robot.isSimulation() && Timer.getFPGATimestamp() - targetSetTimestamp >= 2) return true;

        return AtTarget(targetMap.get(target));
    }

    public Command GoToCommand(ElevatorTarget target)
    {
        if(target == null) return null;

        return new ElevatorGoToCommand(this, target);
    }

    public Command SetTargetCommand(ElevatorTarget target)
    {
        return Commands.runOnce(() -> SetTarget(target), this);
    }

    private boolean AtTarget(double target)
    {
        double currentPos = encoder.getPosition();

        return Math.abs(target-currentPos) < ElevatorConstants.TargetTolerance;
    }

    private void SetTarget(double target)
    {
        controller.setReference(target, ControlType.kMAXMotionPositionControl);
    }


    @Override
    public void periodic() {
        SmartDashboard.putString("Elevator Target", target.name());
        SmartDashboard.putBoolean("Elevator At Target", AtTarget(target));
    }
}
