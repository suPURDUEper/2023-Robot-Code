package org.supurdueper.frc2023.subsystems.Armavator;

import org.littletonrobotics.junction.AutoLog;

public interface ArmavatorIO {
  @AutoLog
  public static class ArmavatorIOInputs {
    public double armPosition = 0.0;
    public double armVelocity = 0.0;
    public double armAppliedVolts = 0.0;
    public double[] armCurrentAmps = new double[] {};
    public double[] armTemp = new double[] {};

    public double elevatorPosition = 0.0;
    public double elevatorVelocity = 0.0;
    public double elevatorAppliedVolts = 0.0;
    public double[] elevatorCurrentAmps = new double[] {};
    public double[] elevatorTemp = new double[] {};
  }
  /* Updates the sets of loggable inputs */
  public default void updateArmavatorInputs(ArmavatorIOInputs Inputs) {}
  /* Run the arm motor at a specifc Voltage */
  public default void setArmVoltage(double volts) {}
  /* Run the elevator motor at a specifc Voltage */
  public default void setElevatorVoltage(double volts) {}
  /* enable/disable brake mode for arm motor */
  public default void setArmBrakeMode(boolean enable) {}
  /* enable/disable brake mode for elevator motor */
  public default void setElevatorBrakeMode(boolean enable) {}
}
