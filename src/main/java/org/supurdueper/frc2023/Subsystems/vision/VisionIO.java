package org.supurdueper.frc2023.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

  @AutoLog
  public static class VisionIOInputs {
    public Pose2d visionPose = new Pose2d();
    public int targetsInView = 0;
    public double lastCaptureLatencyMs = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs) {}
}
