package org.supurdueper.frc2023.oi;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** Class for controlling the robot with a single Xbox controller. */
public class SingleHandheldOI extends HandheldOI {
  private final CommandXboxController controller;

  public SingleHandheldOI(int port) {
    controller = new CommandXboxController(port);
  }

  @Override
  public double getLeftDriveX() {
    return -controller.getLeftY();
  }

  @Override
  public double getLeftDriveY() {
    return -controller.getLeftX();
  }

  @Override
  public double getRightDriveX() {
    return -controller.getRightY();
  }

  @Override
  public double getRightDriveY() {
    return -controller.getRightX();
  }

  @Override
  public void setDriverRumble(double percent) {
    controller.getHID().setRumble(RumbleType.kRightRumble, percent);
  }

  @Override
  public void setOperatorRumble(double percent) {
    controller.getHID().setRumble(RumbleType.kRightRumble, percent);
  }
}
