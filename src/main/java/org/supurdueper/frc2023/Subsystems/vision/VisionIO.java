package org.supurdueper.frc2023.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.frc2023.util.PoseEstimator.TimestampedVisionUpdate;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface VisionIO {

  public static class VisionIOInputs implements LoggableInputs {
    public TimestampedVisionUpdate visionUpdate;
    public int targetsInView = 0;
    public double lastCaptureLatencyMs = 0.0;

    @Override
    public void toLog(LogTable table) {
      if (visionUpdate != null) {
        table.put(
            "Pose",
            new double[] {
              visionUpdate.pose().getX(),
              visionUpdate.pose().getY(),
              visionUpdate.pose().getRotation().getRadians()
            });
        table.put("Timestamp", visionUpdate.timestamp());
        table.put("TargetsInView", targetsInView);
        table.put("Latency", lastCaptureLatencyMs);
      }
    }

    @Override
    public void fromLog(LogTable table) {
      try {
        targetsInView = (int) table.getInteger("TargetsInView", -1);
        lastCaptureLatencyMs = table.getDouble("Latency", -1);
        double[] visionMeasurements = table.getDoubleArray("Pose", new double[] {-1, -1, -1});
        Pose2d visionPose =
            new Pose2d(
                visionMeasurements[0],
                visionMeasurements[1],
                Rotation2d.fromRadians(visionMeasurements[2]));
        double timestamp = table.getDouble("Timestamp", -1);
        visionUpdate =
            new TimestampedVisionUpdate(timestamp, visionPose, VisionIOLimelight.stdDevs);
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs) {}
}
