package org.supurdueper.frc2023.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Interface for all driver and operator controls (either single or dual Xbox). */
public class HandheldOI {
  protected static final Trigger dummyTrigger = new Trigger(() -> false);

  public double getLeftDriveX() {
    return 0.0;
  }

  public double getLeftDriveY() {
    return 0.0;
  }

  public double getRightDriveX() {
    return 0.0;
  }

  public double getRightDriveY() {
    return 0.0;
  }

  public void setDriverRumble(double percent) {}

  public void setOperatorRumble(double percent) {}
}
