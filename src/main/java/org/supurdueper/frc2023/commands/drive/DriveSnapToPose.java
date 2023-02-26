package org.supurdueper.frc2023.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.util.function.Supplier;
import org.littletonrobotics.frc2023.commands.DriveToPose;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;

public class DriveSnapToPose extends DriveToPose {

  private static final double maxXAdjustmentMeters = Units.inchesToMeters(12);
  private static final double maxYAdjustmentMeters = Units.inchesToMeters(12);

  /** Automatically snap the drive to a pose, but allow the driver to slightly adjust the robot. */
  public DriveSnapToPose(
      Drive drive,
      Pose2d pose,
      Supplier<Double> xOffsetSupplier,
      Supplier<Double> yOffsetSupplier) {
    super(
        drive,
        () -> {
          double xAdjustmentMeters = xOffsetSupplier.get() * maxXAdjustmentMeters;
          double yAdjustmentMeters = yOffsetSupplier.get() * maxYAdjustmentMeters;
          return transformPoseByOffset(pose, xAdjustmentMeters, yAdjustmentMeters);
        });
  }

  public DriveSnapToPose(
      Drive drive,
      Supplier<Pose2d> pose,
      Supplier<Double> xOffsetSupplier,
      Supplier<Double> yOffsetSupplier) {
    super(
        drive,
        () -> {
          double xAdjustmentMeters = xOffsetSupplier.get() * maxXAdjustmentMeters;
          double yAdjustmentMeters = yOffsetSupplier.get() * maxYAdjustmentMeters;
          return transformPoseByOffset(pose.get(), xAdjustmentMeters, yAdjustmentMeters);
        });
  }

  private static Pose2d transformPoseByOffset(Pose2d pose, double xOffset, double yOffset) {
    Translation2d translation = new Translation2d(xOffset, yOffset);
    Transform2d transform = new Transform2d(translation, new Rotation2d());
    return pose.transformBy(transform);
  }
}
