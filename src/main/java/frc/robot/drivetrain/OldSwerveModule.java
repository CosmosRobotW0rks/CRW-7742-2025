package frc.robot.drivetrain;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Time;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class OldSwerveModule {

    private SparkMax angleSpark;
    private SparkMax driveSpark;

    private RelativeEncoder angleEncoder;
    private RelativeEncoder driveEncoder;

    private SparkClosedLoopController angleController;
    private SparkClosedLoopController driveController;

    double distanceDriven;

    Time lastUpdateTime;

    boolean inverted;
    double targetAngleAbsolute;

    public OldSwerveModule(int AngleCANID, int DriveCANID) {
        
        angleSpark = new SparkMax(AngleCANID, MotorType.kBrushless);
        driveSpark = new SparkMax(DriveCANID, MotorType.kBrushless);
        
        SparkMaxConfig angleConfig = new SparkMaxConfig();
        SparkMaxConfig driveConfig = new SparkMaxConfig();

        double angleConvFactor = 2.0 * Math.PI / SwerveConstants.GearRatio_Angle;

        angleConfig.closedLoop.pid(SwerveConstants.AnglePID_P, SwerveConstants.AnglePID_I, SwerveConstants.AnglePID_D);
        angleConfig.closedLoop.outputRange(-0.5,0.5);
        angleConfig.idleMode(IdleMode.kBrake);

        angleConfig.encoder.positionConversionFactor(angleConvFactor);


            


        driveConfig.closedLoop.pid(SwerveConstants.DrivePID_P, SwerveConstants.DrivePID_I, SwerveConstants.DrivePID_D);
        driveConfig.idleMode(IdleMode.kCoast);

        angleSpark.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        driveSpark.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);





        angleEncoder = angleSpark.getEncoder();
        driveEncoder = driveSpark.getEncoder();
        angleController = angleSpark.getClosedLoopController();
        driveController = driveSpark.getClosedLoopController();

        
    }


    public double GetAngle() // Radians
    {
        return angleEncoder.getPosition();
    }

    public double GetShortestRouteToAngle(double currentAngle, double targetAngle)
    {

        double currentAngle_clamped = currentAngle % (2.0 * Math.PI);
        currentAngle_clamped += currentAngle_clamped < 0 ? 2.0 * Math.PI : 0;

        double nt = targetAngle + currentAngle - currentAngle_clamped;

        if (targetAngle - currentAngle_clamped > Math.PI)
            nt -= 2.0 * Math.PI;

        if (targetAngle - currentAngle_clamped < -Math.PI)
            nt += 2.0 * Math.PI;

        return nt;
    }

    public void Drive(double speed) 
    {
        double remainingAngleDistance = angleEncoder.getPosition() * targetAngleAbsolute;

        double speedToApply = speed * remainingAngleDistance;
        
        SetSpeed(speedToApply);
    }

    public void SetTargetAngle(double angDeg, boolean forceForward)
    {
        double currentAngle = GetAngle();

        double currentAngle_clamped = currentAngle % (2.0 * Math.PI);
        currentAngle_clamped += currentAngle_clamped < 0 ? 2.0 * Math.PI : 0;

        double angle = Math.toRadians(angDeg);

        double target1 = GetShortestRouteToAngle(currentAngle, angle);
        double target2 = GetShortestRouteToAngle(currentAngle, angle > Math.PI ? angle - Math.PI : angle + Math.PI);

        double route1Diff = Math.abs(target1 - currentAngle);
        double route2Diff = Math.abs(target2 - currentAngle);
        

        if(!forceForward && route1Diff > route2Diff)
        {
            inverted = false;
            targetAngleAbsolute = target2;
        }
        else
        {
            inverted = true;
            targetAngleAbsolute = target1;
        }

        angleController.setReference(targetAngleAbsolute, ControlType.kPosition);

    }
    
    public void SetTargetAngle(double angleDegrees)
    {
        SetTargetAngle(angleDegrees, false);
    }

    public double GetSpeed() // unit m/s
    {
        double rpm = driveEncoder.getVelocity();

        return rpm / 250.0 / SwerveConstants.SpeedCalibValue;
    }

    void SetSpeed(double speed)
    {
        driveController.setReference(speed * 250.0 * SwerveConstants.SpeedCalibValue * (inverted ? -1 : 1), ControlType.kVelocity);
    }

    public void Update(double deltaTimeSeconds)
    {
        distanceDriven += GetSpeed() * deltaTimeSeconds;
    }

    public SwerveModulePosition GetPosition()
    {
        return new SwerveModulePosition(distanceDriven, Rotation2d.fromRadians(GetAngle()));
    }

    public SwerveModuleState GetState()
    {
        return new SwerveModuleState(GetSpeed(), Rotation2d.fromRadians(GetAngle()));
    }

}
