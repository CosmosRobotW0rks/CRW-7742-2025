package frc.robot.drivetrain;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    
    private SparkMax angleSpark;
    private SparkMax driveSpark;

    private RelativeEncoder angleEncoder;
    private RelativeEncoder driveEncoder;

    private SparkClosedLoopController angleController;
    private SparkClosedLoopController driveController;


    
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


            


        driveConfig.closedLoop.pid(SwerveConstants.DrivePIDF_P, SwerveConstants.DrivePIDF_I, SwerveConstants.DrivePIDF_D);
        driveConfig.idleMode(IdleMode.kCoast);

        angleSpark.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        driveSpark.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);





        angleEncoder = angleSpark.getEncoder();
        driveEncoder = driveSpark.getEncoder();
        angleController = angleSpark.getClosedLoopController();
        driveController = driveSpark.getClosedLoopController();
    }
}
