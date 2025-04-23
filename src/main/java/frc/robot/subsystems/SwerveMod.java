package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.PIDValues;

public class SwerveMod {
    public SparkMax DriveMotor, RotationMotor; //Two motors
    public RelativeEncoder DriveEncoder; // You can get various values from the encoder like velocity, position etc.
    public CANcoder cancoder; //CTRE CANCoder
    public SparkClosedLoopController DrivePID; //Driving PID control for smoothen drive
    public PIDController RotationPID; //Rotation PID for facing tuning
    public CANcoderConfiguration cancoderConfig; //CANCoder configuration
    public SparkMaxConfig Driveconfig, RotationConfig; // Motor Configuration


    public SwerveMod(int DriveMotorID, int RotationMotorID, int CANCoderID, double CANCoderOffset){
        //initial the item we just defined
        DriveMotor = new SparkMax(DriveMotorID, MotorType.kBrushless);
        RotationMotor = new SparkMax(RotationMotorID, MotorType.kBrushless);
        cancoder = new CANcoder(CANCoderID);

        cancoderConfig = new CANcoderConfiguration();
        Driveconfig = new SparkMaxConfig();
        RotationConfig = new SparkMaxConfig();

        Driveconfig
            .idleMode(IdleMode.kBrake)
            .inverted(false);
        //closedloop means PID control, not required but you can smoothen your drive when applied.
        Driveconfig.closedLoop
            .pidf(PIDValues.DrivePID[0], PIDValues.DrivePID[1], PIDValues.DrivePID[2], PIDValues.DrivePID[3])
            .outputRange(-1, 1);

        //configure the encoder
        Driveconfig.encoder
            .positionConversionFactor(MotorConstants.kDrivePositionConversionFactor)
            .velocityConversionFactor(MotorConstants.kDriveVelocityConversionFactor);

        RotationConfig
            .idleMode(IdleMode.kBrake)
            .inverted(true);
        RotationConfig.encoder
            .positionConversionFactor(MotorConstants.kDrivePositionConversionFactor)
            .velocityConversionFactor(MotorConstants.kDriveVelocityConversionFactor);

        cancoderConfig.MagnetSensor
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            .withMagnetOffset(CANCoderOffset);


        //apply the motor settings to the motor
        DriveMotor.configure(Driveconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        RotationMotor.configure(RotationConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        cancoder.getConfigurator().apply(cancoderConfig); //applying settings to CANCoder

        DriveEncoder = DriveMotor.getEncoder();//get encoder from motor
        RotationPID = new PIDController(PIDValues.RotationPID[0], PIDValues.RotationPID[1], PIDValues.RotationPID[2]);//Define a PID controller for rotation
        DrivePID = DriveMotor.getClosedLoopController();// get drive hardware PID controller
    }

    //get the module current state (v, θ) in (m/s, deg)
    public SwerveModuleState getState(){
        return new SwerveModuleState(
            DriveEncoder.getVelocity(),
            Rotation2d.fromDegrees(cancoder.getPosition().getValueAsDouble())
        );
    }

    //get the module position for odometry in (p, θ) of (m, deg)
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            DriveEncoder.getPosition(),
            Rotation2d.fromDegrees(cancoder.getPosition().getValueAsDouble())
        );
    }

    //set the module state to the desired state
    public void setState(SwerveModuleState state){
        state.optimize(getState().angle);
        DrivePID.setReference(mpsToRPM(state.speedMetersPerSecond), ControlType.kVelocity);
        RotationMotor.set(
            RotationPID.calculate(
                Rotation2d.fromRotations(cancoder.getAbsolutePosition().getValueAsDouble()).getDegrees(),
                state.angle.getDegrees())
        );
    }

    //converter
    public double mpsToRPM(double mps){
        return (mps*60)/(ChassisConstants.kWheelDiameter*Math.PI)*ChassisConstants.kDriveGearRatio;
    }
}
