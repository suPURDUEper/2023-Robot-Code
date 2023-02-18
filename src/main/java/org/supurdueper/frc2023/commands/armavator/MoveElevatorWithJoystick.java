// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.frc2023.commands.armavator;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;
import org.supurdueper.frc2023.subsystems.armavator.Armavator;

public class MoveElevatorWithJoystick extends CommandBase {

  Supplier<Double> joystickValue;
  Armavator armavator;

  public MoveElevatorWithJoystick(Armavator armavator, Supplier<Double> joystickValue) {
    this.joystickValue = joystickValue;
    this.armavator = armavator;
    addRequirements(armavator);
  }

  @Override
  public void execute() {
    armavator.setElevatorVoltage(joystickValue.get() * RobotController.getBatteryVoltage());
  }
}
