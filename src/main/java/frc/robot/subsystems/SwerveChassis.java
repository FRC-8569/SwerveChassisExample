package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.MotorConstants;

public class SwerveChassis extends SubsystemBase{
    public SwerveMod LeftFront, RightFront, LeftBack, RightBack; //Define the modules
    public AHRS gyro;
    public SwerveDriveOdometry odometry;
    public StructArrayPublisher<SwerveModuleState> statePublish; //For AdvantageScope (Data publishing)
    public StructPublisher<Pose2d> odometryPublish; //and also this line

    public SwerveChassis(){

        //Initialize the items
        LeftFront = new SwerveMod(
            MotorConstants.LeftFrontDriveID, 
            MotorConstants.LeftFrontRotationID, 
            MotorConstants.LeftFrontCANCoderID, 
            MotorConstants.LeftFrontOffset);

        LeftBack = new SwerveMod(
            MotorConstants.LeftBackDriveID, 
            MotorConstants.LeftBackRotationID, 
            MotorConstants.LeftBackCANCoderID , 
            MotorConstants.LeftBackOffset);

        RightFront = new SwerveMod(
            MotorConstants.RightFrontDriveID, 
            MotorConstants.RightFrontRotationID, 
            MotorConstants.RightFrontCANCoderID, 
            MotorConstants.RightFrontOffset);

        RightBack = new SwerveMod(
            MotorConstants.RightBackRotationID, 
            MotorConstants.RightBackRotationID, 
            MotorConstants.RightBackCANCoderID, 
            MotorConstants.RightBackOffset);

        gyro = new AHRS(NavXComType.kMXP_SPI);
        odometry = new SwerveDriveOdometry(ChassisConstants.kSwerveDriveKinematics, gyro.getRotation2d(), getModulePosition());
        statePublish = NetworkTableInstance.getDefault().getStructArrayTopic("SwerveDrive",SwerveModuleState.struct).publish(PubSubOption.keepDuplicates(false));
        odometryPublish = NetworkTableInstance.getDefault().getStructTopic("Odometry", Pose2d.struct).publish(PubSubOption.keepDuplicates(false));
    }

    //drive the chassis from the double value input
    public void drive(double xSpeed, double ySpeed, double zRotation){
        setModuleState(
            ChassisConstants.kSwerveDriveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromRobotRelativeSpeeds(ChassisSpeeds.discretize(new ChassisSpeeds(xSpeed, ySpeed, zRotation), 0.02), gyro.getRotation2d())
            )
        );
    }

    //receive the module states from the SwerveModuleStates and apply to each module
    public void setModuleState(SwerveModuleState[] states){
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MotorConstants.kMaxRobotSpeedMPS);
        LeftFront.setState(states[0]);
        RightFront.setState(states[1]);
        LeftBack.setState(states[2]);
        RightBack.setState(states[3]);
    }

    //get the module positions
    public SwerveModulePosition[] getModulePosition(){
        return new SwerveModulePosition[]{
            LeftFront.getPosition(),
            RightFront.getPosition(),
            LeftBack.getPosition(),
            RightBack.getPosition()
        };
    }

    public SwerveModuleState[] getModuleStates(){
        return new SwerveModuleState[]{
            LeftFront.getState(),
            RightFront.getState(),
            LeftBack.getState(),
            RightBack.getState()
        };
    }

    //upload the data to the NetworkTables
    @Override
    public void periodic(){
        odometry.update(gyro.getRotation2d(), getModulePosition());
        odometryPublish.set(odometry.getPoseMeters());
        statePublish.set(this.getModuleStates());
    }
}
