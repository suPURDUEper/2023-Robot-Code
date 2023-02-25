// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.frc2023.commands.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;
import org.supurdueper.frc2023.subsystems.elevator.Elevator;

public class MoveElevatorWithJoystick extends CommandBase {

  Supplier<Double> joystickValue;
  Elevator elevator;
  double elevatorKg;

  public MoveElevatorWithJoystick(Elevator elevator, Supplier<Double> joystickValue) {
    this.joystickValue = joystickValue;
    this.elevator = elevator;
    addRequirements(elevator);
    this.elevatorKg = Elevator.elevatorKg;
  }

  @Override
  public void execute() {
    double filteredJoystickValue = MathUtil.applyDeadband(joystickValue.get(), 0.2);
    // Not calling set votlage will let the PID loop keep running to hold the elevator in place
    // if the joystick isn't touched
    if (elevator.inputs.isElevatorRunningPID && filteredJoystickValue == 0) {
      // Do nothing
    } else {
      elevator.setVoltage((filteredJoystickValue * RobotController.getBatteryVoltage()));
    }
  }
}
