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
    public SparkMax DriveMotor, RotationMotor;
    public RelativeEncoder DriveEncoder;
    public CANcoder cancoder;
    public SparkClosedLoopController DrivePID;
    public PIDController RotationPID;
    public CANcoderConfiguration cancoderConfig;
    public SparkMaxConfig Driveconfig, RotationConfig;


    public SwerveMod(int DriveMotorID, int RotationMotorID, int CANCoderID, double CANCoderOffset){
        DriveMotor = new SparkMax(DriveMotorID, MotorType.kBrushless);
        RotationMotor = new SparkMax(RotationMotorID, MotorType.kBrushless);
        cancoder = new CANcoder(CANCoderID);

        cancoderConfig = new CANcoderConfiguration();
        Driveconfig = new SparkMaxConfig();
        RotationConfig = new SparkMaxConfig();

        Driveconfig
            .idleMode(IdleMode.kBrake)
            .inverted(false);
        Driveconfig.closedLoop
            .pidf(PIDValues.DrivePID[0], PIDValues.DrivePID[1], PIDValues.DrivePID[2], PIDValues.DrivePID[3])
            .outputRange(-1, 1);
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

        DriveMotor.configure(Driveconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        RotationMotor.configure(RotationConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        cancoder.getConfigurator().apply(cancoderConfig);

        DriveEncoder = DriveMotor.getEncoder();
        RotationPID = new PIDController(PIDValues.RotationPID[0], PIDValues.RotationPID[1], PIDValues.RotationPID[2]);
        DrivePID = DriveMotor.getClosedLoopController();
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            DriveEncoder.getVelocity(),
            Rotation2d.fromDegrees(cancoder.getPosition().getValueAsDouble())
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            DriveEncoder.getPosition(),
            Rotation2d.fromDegrees(cancoder.getPosition().getValueAsDouble())
        );
    }

    public void setState(SwerveModuleState state){
        state.optimize(getState().angle);
        DrivePID.setReference(mpsToRPM(state.speedMetersPerSecond), ControlType.kVelocity);
        RotationMotor.set(
            RotationPID.calculate(
                Rotation2d.fromRotations(cancoder.getAbsolutePosition().getValueAsDouble()).getDegrees(),
                state.angle.getDegrees())
        );
    }

    public double mpsToRPM(double mps){
        return (mps*60)/(ChassisConstants.kWheelDiameter*Math.PI)*ChassisConstants.kDriveGearRatio;
    }
}
