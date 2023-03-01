package org.supurdueper.frc2023.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import java.util.function.Supplier;
import org.littletonrobotics.frc2023.commands.DriveToPose;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.util.GeomUtil;

public class DriveSnapToPose extends DriveToPose {

  private static final double maxXAdjustmentMeters = Units.inchesToMeters(12);
  private static final double maxYAdjustmentMeters = Units.inchesToMeters(12);

  /** Automatically snap the drive to a pose, but allow the driver to slightly adjust the robot. */
  public DriveSnapToPose(
      Drive drive,
      Supplier<Pose2d> pose,
      Supplier<Double> xOffsetSupplier,
      Supplier<Double> yOffsetSupplier) {
    super(
        drive,
        () -> {
          double xOffset = xOffsetSupplier.get() * maxXAdjustmentMeters;
          double yOffset = yOffsetSupplier.get() * maxYAdjustmentMeters;
          return pose.get().transformBy(GeomUtil.translationToTransform(xOffset, yOffset));
        });
  }

  public DriveSnapToPose(
      Drive drive,
      Pose2d pose,
      Supplier<Double> xOffsetSupplier,
      Supplier<Double> yOffsetSupplier) {
    this(drive, () -> pose, xOffsetSupplier, yOffsetSupplier);
  }
}
