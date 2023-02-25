package org.supurdueper.frc2023.subsystems.arm;

public interface ArmMotorIO {
  // @AutoLog
  public static class ArmMotorIOInputs {
    public double armPositionRad = 0.0;
    public double armVelocityRadS = 0.0;
    public double armAppliedVolts = 0.0;
    public double armTargetPositionRad = 0.0;
    public double armTargetVelocityRadS = 0.0;
    public double armFeedforward = 0.0;
    public double[] armCurrentAmps = new double[] {};
    public double[] armTemp = new double[] {};
    public boolean isArmRunningPID = false;
    public double outputPID = 0.0;
  }
  /* Updates the sets of loggable inputs */
  public default void updateInputs(ArmMotorIOInputs Inputs) {}
  /* Run the elevator motor at a specifc Voltage */
  public default void setVoltage(double volts) {}
  /* enable/disable brake mode for elevator motor */
  public default void setBrakeMode(boolean enable) {}
  /* Set PID Gains for the elevator */
  public default void setPIDGains(double kP, double kI, double kD) {}
}
