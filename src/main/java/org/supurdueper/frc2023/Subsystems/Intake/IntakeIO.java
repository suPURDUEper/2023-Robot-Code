package org.supurdueper.frc2023.subsystems.intake;

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

  /** Set the intake roller voltage */
  public default void setRollerVoltage(double volts) {}


  public default double getRollerAmps() {
    return 0;
  }

  public default void setCurrentLimit(
      double currentLimit, double triggerAmps, double triggerTimeSeconds) {}
}
