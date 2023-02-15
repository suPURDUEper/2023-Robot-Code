package org.supurdueper.frc2023.Subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class CubeIntakeIOInputs {
    public double armAbsolutePositionRad = 0.0;
    public double armRelativePositionRad = 0.0;
    public double armInternalPositionRad = 0.0;
    public double armRelativeVelocityRadPerSec = 0.0;
    public double armInternalVelocityRadPerSec = 0.0;
    public double armAppliedVolts = 0.0;
    public double[] armCurrentAmps = new double[] {};
    public double[] armTempCelcius = new double[] {};

    public double rollerAppliedVolts = 0.0;
    public double[] rollerCurrentAmps = new double[] {};
  }
  /** Updates the set of loggable inputs. */
  public default void updateInputs(CubeIntakeIOInputs inputs) {}

  /** Set the arm motor voltage */
  public default void setArmVoltage(double volts) {}

  /** Set the intake roller voltage */
  public default void setRollerVoltage(double volts) {}
}