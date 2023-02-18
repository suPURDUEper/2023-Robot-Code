package org.supurdueper.frc2023.subsystems.armavator;

import org.littletonrobotics.junction.AutoLog;

public interface ArmavatorMotorIO {
  @AutoLog
  public static class ArmavatorMotorIOInputs {
    public double armPositionRad = 0.0;
    public double armVelocityRadS = 0.0;
    public double armAppliedVolts = 0.0;
    public double armTargetPositionRad = 0.0;
    public double armTargetVelocityRadS = 0.0;
    public double armFeedforward = 0.0;
    public double[] armCurrentAmps = new double[] {};
    public double[] armTemp = new double[] {};
    public boolean isArmRunningPID = false;

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
  public default void updateArmavatorInputs(ArmavatorMotorIOInputs Inputs) {}
  /* Run the arm motor at a specifc Voltage */
  public default void setArmVoltage(double volts) {}
  /* Run the elevator motor at a specifc Voltage */
  public default void setElevatorVoltage(double volts) {}
  /* enable/disable brake mode for arm motor */
  public default void setArmBrakeMode(boolean enable) {}
  /* enable/disable brake mode for elevator motor */
  public default void setElevatorBrakeMode(boolean enable) {}
  /* Set PID Gains for the arm */
  public default void setArmPIDGains(double kP, double kI, double kD) {}
  /* Set PID Gains for the elevator */
  public default void setElevatorPIDGains(double kP, double kI, double kD) {}
}
