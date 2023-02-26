package org.supurdueper.frc2023.subsystems.vision;

import org.littletonrobotics.frc2023.util.PoseEstimator.TimestampedVisionUpdate;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

  @AutoLog
  public static class VisionIOInputs {
    public TimestampedVisionUpdate visionUpdate;
    public int targetsInView = 0;
    public double lastCaptureLatencyMs = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs) {}
}
