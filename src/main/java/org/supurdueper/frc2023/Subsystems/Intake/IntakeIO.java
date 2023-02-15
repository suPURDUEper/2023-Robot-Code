package org.supurdueper.frc2023.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {

    public double rollerAppliedVolts = 0.0;
    public double rollerCurrentAmps = 0.0;
    public double rollerTempCelcius = 0.0;
  }
  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Set the arm motor voltage */
  public default void setArmVoltage(double volts) {}

  /** Set the intake roller voltage */
  public default void setRollerVoltage(double volts) {}
}
