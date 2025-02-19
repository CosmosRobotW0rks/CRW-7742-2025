// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;

public final class Constants {


    public static class DriveConstants {
        
        public static final double MaxDriveSpeed = 2; // m/s
        public static final double MaxDriveAccel = 10; // m/s^2

        public static final double MaxRotSpeed = Math.PI * 2 * 0.5; // rad/s
        public static final double MaxRotAccel = Math.PI * 2 * 6; // rad/s^2

        public static final double JOYDeadzone_X = 0.1;
        public static final double JOYDeadzone_Y = 0.1;
        public static final double JOYDeadzone_Rot = 0.1;

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
        public static final double TARGET_PROCESSOR = 0;
        public static final double TARGET_REEFL1 = 0;
        public static final double TARGET_REEFL2 = 0;
        public static final double TARGET_REEFL3 = 0;
        public static final double TARGET_REEFL4 = 0;

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


    }
}
