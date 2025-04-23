// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ChassisCmd;
import frc.robot.subsystems.SwerveChassis;

public class RobotContainer {
  public SwerveChassis chassis = new SwerveChassis();
  public Joystick joystick = new Joystick(0);

  public RobotContainer() {
    chassis.setDefaultCommand(new ChassisCmd(
      chassis, 
      () -> joystick.getRawAxis(1), 
      () -> joystick.getRawAxis(0), 
      () -> joystick.getRawAxis(2)));
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
