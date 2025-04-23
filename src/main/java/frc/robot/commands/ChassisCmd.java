package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveChassis;

public class ChassisCmd extends Command{
    public SwerveChassis chassis;
    public Supplier<Double> xSpeedFunc, ySpeedFunc, zRotationFunc;

    public ChassisCmd(SwerveChassis chassis, Supplier<Double> xSpeeed, Supplier<Double> ySpeed, Supplier<Double> zRotation){
        this.chassis = chassis;
        this.xSpeedFunc = xSpeeed;
        this.ySpeedFunc = ySpeed;
        this.zRotationFunc = zRotation;

        addRequirements(chassis);
    }

    @Override
    public void execute(){
        chassis.drive(xSpeedFunc.get(), ySpeedFunc.get(), zRotationFunc.get());
    }
}
