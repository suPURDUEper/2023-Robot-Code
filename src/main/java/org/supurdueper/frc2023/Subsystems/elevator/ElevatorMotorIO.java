package org.supurdueper.frc2023.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorMotorIO {
  @AutoLog
  public static class ElevatorMotorIOInputs {

    public double elevatorPositionM = 0.0;
    public double elevatorVelocityMS = 0.0;
    public double elevatorAppliedVolts = 0.0;
    public double elevatorTargetPositionM = 0.0;
    public double elevatorTargetVelocityMS = 0.0;
    public double elevatorFeedforward = 0.0;
    public double[] elevatorCurrentAmps = new double[] {};
    public double[] elevatorTemp = new double[] {};
    public boolean isElevatorRunningPID = false;
  }
  /* Updates the sets of loggable inputs */
  public default void updateInputs(ElevatorMotorIOInputs Inputs) {}
  /* Run the elevator motor at a specifc Voltage */
  public default void setVoltage(double volts) {}
  /* enable/disable brake mode for elevator motor */
  public default void setBrakeMode(boolean enable) {}
  /* Set PID Gains for the elevator */
  public default void setPIDGains(double kP, double kI, double kD) {}
  /*  */
  public default void resetEncoder() {}
  ;
}
