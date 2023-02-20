package org.supurdueper.frc2023.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  // @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public double rollPositionRad = 0.0;
    public double pitchPositionRad = 0.0;
    public double yawPositionRad = 0.0;
    public double rollVelocityRadPerSec = 0.0;
    public double pitchVelocityRadPerSec = 0.0;
    public double yawVelocityRadPerSec = 0.0;
  }

  public default void updateInputs(GyroIOInputs inputs) {}
}
