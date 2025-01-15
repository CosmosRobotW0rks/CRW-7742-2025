package frc.robot.drivetrain;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotation;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Distance;

import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.SwerveConstants;

public class SwerveModule {

    private SparkMax angleSpark;
    private SparkMax driveSpark;

    private RelativeEncoder angleEncoder;
    private RelativeEncoder driveEncoder;

    private SparkClosedLoopController angleController;
    private SparkClosedLoopController driveController;

    public SwerveModule(int AngleCANID, int DriveCANID) {

        angleSpark = new SparkMax(AngleCANID, MotorType.kBrushless);
        driveSpark = new SparkMax(DriveCANID, MotorType.kBrushless);

        SparkMaxConfig angleConfig = new SparkMaxConfig();
        SparkMaxConfig driveConfig = new SparkMaxConfig();

        double anglePCF = 2.0 * Math.PI / SwerveConstants.GearRatio_Angle; // Output: rad
        double drivePCF = SwerveConstants.GearRatio_Drive / SwerveConstants.WheelDiameterM * Math.PI; // Output: m
        double driveVCF = 60 * drivePCF; // Output: m/s

        angleConfig.idleMode(IdleMode.kCoast);
        driveConfig.idleMode(IdleMode.kCoast);

        angleConfig.closedLoop.pid(SwerveConstants.AnglePID_P, SwerveConstants.AnglePID_I, SwerveConstants.AnglePID_D);
        driveConfig.closedLoop.pidf(SwerveConstants.DrivePIDF_P, SwerveConstants.DrivePIDF_I,
                SwerveConstants.DrivePIDF_D, SwerveConstants.DrivePIDF_FF);

        angleConfig.closedLoop.outputRange(-0.5, 0.5);

        angleConfig.encoder.positionConversionFactor(anglePCF);
        driveConfig.encoder.positionConversionFactor(drivePCF);
        driveConfig.encoder.velocityConversionFactor(driveVCF);

        angleSpark.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        driveSpark.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        angleEncoder = angleSpark.getEncoder();
        driveEncoder = driveSpark.getEncoder();
        angleController = angleSpark.getClosedLoopController();
        driveController = driveSpark.getClosedLoopController();
    }

    public double GetDistanceM() {
        return driveEncoder.getPosition();
    }

    public Rotation2d GetAngle() {
        return Rotation2d.fromRadians(angleEncoder.getPosition());
    }

    public double GetSpeedMPS() {
        return driveEncoder.getVelocity();
    }

    public void SetTargetAngle(Rotation2d angle) {
        angleController.setReference(angle.getRadians(), ControlType.kPosition);
    }

    public void SetTargetSpeedMPS(double speed) {
        driveController.setReference(speed, ControlType.kVelocity);
    }

    public SwerveModuleState GetState() {
        return new SwerveModuleState(GetSpeedMPS(), GetAngle());
    }

    public SwerveModulePosition GetPosition()
    {
        return new SwerveModulePosition(Meters.of(GetDistanceM()), GetAngle());
    }

    public void SetTargetState(SwerveModuleState state) {
        SetTargetSpeedMPS(state.speedMetersPerSecond);
        SetTargetAngle(state.angle);
    }
}
