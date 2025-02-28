// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.autoalign.AutoHelper.ReefAlign;

public final class Constants {



    public static class VisionConstants {
        public static final Transform3d RobotToCam = new Transform3d(new Translation3d(0.35, 0.0, 0.15), new Rotation3d(0,45,0));
    }

    public static class ElevatorConstants {

        public static final int CANID = 0;

        public static final double PID_P = 0;
        public static final double PID_I = 0;
        public static final double PID_D = 0;
        
        public static final double PCF = 0;

        public static final double TargetTolerance = 0;
        public static final double TargetTimeoutS = 0;

        public static final double TARGET_CORALSTAT = 0;
        public static final double TARGET_REEFL1 = 0;
        public static final double TARGET_REEFL2 = 0;
        public static final double TARGET_REEFL3 = 0;
        public static final double TARGET_REEFL4 = 0;
        public static final double TARGET_REEFALGAE1 = 0;
        public static final double TARGET_REEFALGAE2 = 0;

    }

    public static class AutoConstants {
        public static final double FineAlignDrive_kP = 5;
        public static final double FineAlignDrive_kI = 0;
        public static final double FineAlignDrive_kD = 0;

        public static final double FineAlignAngle_kP = 3;
        public static final double FineAlignAngle_kI = 0;
        public static final double FineAlignAngle_kD = 0;

        public static final double FineAlignTolerance_Translation = 0.02; // Meters 
        public static final double FineAlignTolerance_Rotation = (2 * Math.PI) / 120; // Radians

        public static final double FineAlignMaxDriveSpeed = 1; // m/s
        public static final double FineAlignMaxRotSpeed = 2 * Math.PI; // rad/s

        public static final double MaxDriveSpeed = 2.5; // m/s
        public static final double MaxDriveAccel = 5; // m/s^2

        public static final double MaxRotSpeed = Math.PI * 2 * 3; // rad/s
        public static final double MaxRotAccel = Math.PI * 2 * 6; // rad/s^2

        
        public static final Translation2d CoralStationOffset = new Translation2d(0.5, 0);

        public static final HashMap<ReefAlign, Translation2d> ReefAlignOffsets = new HashMap<ReefAlign, Translation2d>() {{
            put(ReefAlign.Left, new Translation2d(0.5, -0.2));
            put(ReefAlign.Center, new Translation2d(0.5, 0));
            put(ReefAlign.Right, new Translation2d(0.5, 0.2));
        }};

    }

    public static class DriveConstants {
        
        public static final double MaxDriveSpeed = 3; // m/s
        public static final double MaxDriveAccel = 5; // m/s^2
        public static final double MaxDriveDeccel = 8; // m/s^2

        public static final double MaxRotSpeed = Math.PI * 2 * 0.5; // rad/s
        public static final double MaxRotAccel = Math.PI * 2 * 6; // rad/s^2
        public static final double MaxRotDeccel = Math.PI * 2 * 6; // rad/s^2
        

        public static final double JOYDeadzone_X = 0.1;
        public static final double JOYDeadzone_Y = 0.1;
        public static final double JOYDeadzone_Rot = 0.1;

    }

    public static class SwerveConstants {

        public static final double ModuleOffsetM_X = 0.37;
        public static final double ModuleOffsetM_Y = 0.37;

        public static final double GearRatio_Angle = 11.7187;
        public static final double GearRatio_Drive = 6.5;
        public static final double WheelDiameterM = Units.inchesToMeters(4);

        public static final double AnglePID_P = 0.8;
        public static final double AnglePID_I = 0;
        public static final double AnglePID_D = 0; 

        public static final double DrivePIDF_P = 0.1;
        public static final double DrivePIDF_I = 0; 
        public static final double DrivePIDF_D = 0;
        public static final double DrivePIDF_FF = 0.3; 



        public static final int AngleCANID_FL = 1;
        public static final int DriveCANID_FL = 2;

        public static final int AngleCANID_FR = 3;
        public static final int DriveCANID_FR = 4;

        public static final int AngleCANID_BL = 5;
        public static final int DriveCANID_BL = 6;

        public static final int AngleCANID_BR = 7;
        public static final int DriveCANID_BR = 8;

        public static final int ABSENCPORTID_FL = 1;
        public static final int ABSENCPORTID_FR = 2;
        public static final int ABSENCPORTID_BL = 3;
        public static final int ABSENCPORTID_BR = 4;
        public static final boolean ABSENCODER_INVERTED = false;
        


    }
}
