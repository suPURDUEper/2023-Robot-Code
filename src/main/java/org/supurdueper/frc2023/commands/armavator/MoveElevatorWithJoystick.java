// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.frc2023.commands.armavator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;
import org.supurdueper.frc2023.subsystems.elevator.Elevator;

public class MoveElevatorWithJoystick extends CommandBase {

  Supplier<Double> joystickValue;
  Elevator elevator;

  public MoveElevatorWithJoystick(Elevator elevator, Supplier<Double> joystickValue) {
    this.joystickValue = joystickValue;
    this.elevator = elevator;
    addRequirements(elevator);
  }

  @Override
  public void execute() {
    double filteredJoystickValue = MathUtil.applyDeadband(joystickValue.get(), 0.05);
    // Not calling set votlage will let the PID loop keep running to hold the elevator in place
    // if the joystick isn't touched
    if (filteredJoystickValue != 0) {
      // Add elevatorKg so it holds position
      elevator.setVoltage(
          joystickValue.get() * RobotController.getBatteryVoltage() + Elevator.elevatorKg.get());
    }
  }
}
