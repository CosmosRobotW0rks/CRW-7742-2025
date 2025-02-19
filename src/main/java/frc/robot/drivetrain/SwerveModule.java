package frc.robot.drivetrain;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotation;

import com.pathplanner.lib.trajectory.SwerveModuleTrajectoryState;
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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

        double anglePCF = (2.0 * Math.PI) / SwerveConstants.GearRatio_Angle; // Output: rad


        double drivePCF = 1 / (SwerveConstants.GearRatio_Drive / (SwerveConstants.WheelDiameterM * Math.PI)); // Output: m


        double driveVCF = drivePCF / 60; // Output: m/s

        SmartDashboard.putNumber("Drive PCF", drivePCF);
        SmartDashboard.putNumber("Drive VCF", driveVCF);
        SmartDashboard.putNumber("Angle PCF", anglePCF);

        angleConfig.idleMode(IdleMode.kBrake);
        driveConfig.idleMode(IdleMode.kBrake);

        angleConfig.closedLoop.pid(SwerveConstants.AnglePID_P, SwerveConstants.AnglePID_I, SwerveConstants.AnglePID_D);
        driveConfig.closedLoop.pidf(SwerveConstants.DrivePIDF_P, SwerveConstants.DrivePIDF_I,
                SwerveConstants.DrivePIDF_D, SwerveConstants.DrivePIDF_FF);

        angleConfig.inverted(true);
        angleConfig.closedLoop.outputRange(-0.5, 0.5);

        angleConfig.encoder.positionConversionFactor(anglePCF);
        driveConfig.encoder.positionConversionFactor(drivePCF);
        driveConfig.encoder.velocityConversionFactor(driveVCF);

        angleSpark.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        driveSpark.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
        optim(state, false);

        SetTargetSpeedMPS(state.speedMetersPerSecond);
        SetTargetAngle(state.angle);
    }

    public void Home()
    {
        SwerveModuleState state = new SwerveModuleState(0, Rotation2d.kZero);

        optim(state, true);

        SetTargetAngle(state.angle);
    }

    
    public double GetShortestRoute(double a1, double a2) {
        // Clamp currentAngle
        double a1_clamped = a1 % (2.0 * Math.PI);
        a1_clamped += a1_clamped < 0 ? 2.0 * Math.PI : 0;

        // Find shortest route
        double nt = a2 + a1 - a1_clamped;
        if (a2 - a1_clamped > Math.PI)
            nt -= 2.0 * Math.PI;
        if (a2 - a1_clamped < -Math.PI)
            nt += 2.0 * Math.PI;

        return nt;
    }

    public void optim(SwerveModuleState state, boolean forceForward) {
        double currentAngle = angleEncoder.getPosition();
        double currentAngleClamped = currentAngle % (2.0 * Math.PI);
        currentAngleClamped += currentAngleClamped < 0 ? 2.0 * Math.PI : 0;

        double t = state.angle.getRadians();
        double t1 = GetShortestRoute(currentAngle, t);
        double t2 = GetShortestRoute(currentAngle, t > Math.PI ? t - Math.PI : t + Math.PI);

        double diff1 = Math.abs(t1 - currentAngle);
        double diff2 = Math.abs(t2 - currentAngle);

        if (!forceForward && diff1 > diff2) {
            state.angle = Rotation2d.fromRadians(t2);

        } else {
            state.angle = Rotation2d.fromRadians(t1);
        }
    }
}
