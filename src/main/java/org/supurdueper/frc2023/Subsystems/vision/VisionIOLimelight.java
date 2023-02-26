package org.supurdueper.frc2023.subsystems.vision;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.frc2023.util.PoseEstimator.TimestampedVisionUpdate;
import org.supurdueper.frc2023.subsystems.vision.LimelightHelpers.Results;

public class VisionIOLimelight implements VisionIO {

  private static final Matrix<N3, N1> stdDevs =
      new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01);

  public VisionIOLimelight() {
    // Where is the limelight relative to the center of the robot?
    // TODO: Set these values
    LimelightHelpers.setCameraPose_RobotSpace("", 0, 0, 0, 0, 0, 0);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    Results results = LimelightHelpers.getLatestResults("").targetingResults;
    inputs.lastCaptureLatencyMs =
        results.latency_capture + results.latency_pipeline + results.latency_jsonParse;
    double[] poseResult = results.botpose_wpiblue; // [x, y, z, roll, pitch, yaw (in degrees)]
    inputs.visionUpdate =
        new TimestampedVisionUpdate(
            Timer.getFPGATimestamp() - Units.millisecondsToSeconds(inputs.lastCaptureLatencyMs),
            new Pose2d(poseResult[0], poseResult[1], Rotation2d.fromDegrees(poseResult[5])),
            stdDevs);
    inputs.targetsInView = results.targets_Fiducials.length;
  }
}
