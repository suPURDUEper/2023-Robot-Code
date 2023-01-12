// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {

  /* Controller Declare */

  private static XboxController driverJoystick = new XboxController(Constants.JoystickConstants.driverJoystickNumber);;
  private static XboxController operatorJoystick = new XboxController(Constants.JoystickConstants.operatorJoystickNumber);

  /* drive controls */

  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */

  private final Trigger driverB = new JoystickButton(driverJoystick, XboxController.Button.kB.value);
  private final Trigger driverLB = new JoystickButton(driverJoystick, XboxController.Button.kLeftBumper.value);

  /* Operator Buttons */

  /* Subsystems */


  public RobotContainer() {

    

    configureBindings();
  }

  private void configureBindings() {
    /* Driver Buttons */

    

    /* Operator Buttons */
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
