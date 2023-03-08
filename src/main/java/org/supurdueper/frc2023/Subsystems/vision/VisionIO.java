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
    public void fromLog(LogTable table) {
      table.put(
          "VisionUpdate/Pose",
          new double[] {
            visionUpdate.pose().getX(),
            visionUpdate.pose().getY(),
            visionUpdate.pose().getRotation().getRadians()
          });
      table.put("VisionUpdate/Timestamp", visionUpdate.timestamp());
      table.put("TargetsInView", targetsInView);
      table.put("Vision Latency (ms)", lastCaptureLatencyMs);
    }

    @Override
    public void toLog(LogTable table) {
      targetsInView = (int) table.getInteger("TargetsInView", targetsInView);
      lastCaptureLatencyMs = table.getDouble("Vision Latency (ms)", lastCaptureLatencyMs);
      double[] visionMeasurements =
          table.getDoubleArray(
              "VisionUpdate/Pose",
              new double[] {
                visionUpdate.pose().getX(),
                visionUpdate.pose().getY(),
                visionUpdate.pose().getRotation().getRadians()
              });
      Pose2d visionPose =
          new Pose2d(
              visionMeasurements[0],
              visionMeasurements[1],
              Rotation2d.fromRadians(visionMeasurements[2]));
      double timestamp = table.getDouble("VisiosnUpdate/Timestamp", visionUpdate.timestamp());
      visionUpdate = new TimestampedVisionUpdate(timestamp, visionPose, VisionIOLimelight.stdDevs);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs) {}
}
