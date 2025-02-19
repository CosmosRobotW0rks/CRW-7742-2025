package frc.robot.elevator;
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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;


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

        config.encoder.positionConversionFactor(ElevatorConstants.PCF);

        spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = spark.getEncoder();
        controller = spark.getClosedLoopController();

        // TARGET CONFIG
        targetMap.put(ElevatorTarget.CORALSTAT, ElevatorConstants.TARGET_CORALSTAT);
        targetMap.put(ElevatorTarget.REEFL1, ElevatorConstants.TARGET_REEFL1);
        targetMap.put(ElevatorTarget.REEFL2, ElevatorConstants.TARGET_REEFL2);
        targetMap.put(ElevatorTarget.REEFL3, ElevatorConstants.TARGET_REEFL3);
        targetMap.put(ElevatorTarget.REEFL4, ElevatorConstants.TARGET_REEFL4);
        targetMap.put(ElevatorTarget.REEFALGAE1, ElevatorConstants.TARGET_REEFALGAE1);
        targetMap.put(ElevatorTarget.REEFALGAE2, ElevatorConstants.TARGET_REEFALGAE2);
    }

    public void SetTarget(double target)
    {
        controller.setReference(target, ControlType.kPosition);
    }

    public void SetTarget(ElevatorTarget elevatorTarget)
    {
        SetTarget(targetMap.get(elevatorTarget));
    }


    public boolean AtTarget(double target)
    {
        double currentPos = encoder.getPosition();

        return Math.abs(target-currentPos) < ElevatorConstants.TargetTolerance;
    }

    public boolean AtTarget(ElevatorTarget target)
    {
        return AtTarget(targetMap.get(target));
    }
}
