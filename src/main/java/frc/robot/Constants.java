package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class Constants {
    public class ChassisConstants {
        public static final double kDriveGearRatio = 6.12;
        public static final double kWheelDiameter = 4*2.54/100;
        
        public static final SwerveDriveKinematics kSwerveDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(-0.315, -0.37),//FrontLeft
            new Translation2d(-0.315, 0.28),//FrontRight
            new Translation2d(.335, -.37),//BackLeft
            new Translation2d(.335, .28)//BackRight
        );
    }

    public class MotorConstants {
        //Drive motor IDs
        public static final int LeftFrontDriveID = 8;
        public static final int LeftBackDriveID = 4;
        public static final int RightFrontDriveID = 5;
        public static final int RightBackDriveID = 1;

        //Rotation motor IDs
        public static final int LeftFrontRotationID = 16;
        public static final int LeftBackRotationID = 3;
        public static final int RightFrontRotationID = 6;
        public static final int RightBackRotationID = 2;

        public static final int LeftFrontCANCoderID = 2;
        public static final int LeftBackCANCoderID = 1;
        public static final int RightFrontCANCoderID = 3;
        public static final int RightBackCANCoderID = 4;

        public static final double LeftFrontOffset = 0.447998046875;
        public static final double LeftBackOffset = 0.462646484375;
        public static final double RightFrontOffset = 0.162841796875;
        public static final double RightBackOffset = 0.216064453125;
        
        public static final double kMaxMotorSpeedRPS = 5676/60; //NEO Motor Free Speed
        public static final double kDrivePositionConversionFactor = 1/ChassisConstants.kDriveGearRatio*ChassisConstants.kWheelDiameter*Math.PI;
        public static final double kDriveVelocityConversionFactor = kDrivePositionConversionFactor/60;
        public static final double kMotorKv = 476; //search from https://docs.revrobotics.com/brushless/neo/v1.1
        public static final double kMaxRobotSpeedMPS = kMaxMotorSpeedRPS/ChassisConstants.kDriveGearRatio*ChassisConstants.kWheelDiameter*Math.PI; //calculate the maximum velocity of the robot
    }

    public class PIDValues {
        public static final double[] DrivePID = {0.000005, 0, 0, 1/MotorConstants.kMotorKv};
        public static final double[] RotationPID = {0.0078,0,0};
    }
}