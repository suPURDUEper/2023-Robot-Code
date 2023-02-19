package org.supurdueper.frc2023.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.supurdueper.frc2023.subsystems.vision.LimelightHelpers.Results;

public class VisionIOLimelight implements VisionIO {
    
  public VisionIOLimelight() {
    // Where is the limelight relative to the center of the robot?
    LimelightHelpers.setCameraPose_RobotSpace("", 0, 0, 0, 0, 0, 0);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    Results results = LimelightHelpers.getLatestResults("").targetingResults;
    inputs.lastCaptureLatencyMs =
        results.latency_capture + results.latency_pipeline + results.latency_jsonParse;
    double[] poseResult = results.botpose_wpiblue; // [x, y, z, roll, pitch, yaw (in degrees)]
    inputs.visionPose =
        new Pose2d(poseResult[0], poseResult[1], Rotation2d.fromDegrees(poseResult[5]));
    inputs.targetsInView = results.targets_Fiducials.length;
  }
}
