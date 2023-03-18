package org.supurdueper.frc2023.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.frc2023.FieldConstants;
import org.littletonrobotics.frc2023.commands.DriveToPose;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;
import org.littletonrobotics.frc2023.util.GeomUtil;

public class AutoAim extends DriveToPose {

  private static final double scoringXOffset = Units.inchesToMeters(20);
  private static final double maxXAdjustmentMeters = Units.inchesToMeters(12);
  private static final double maxYAdjustmentMeters = Units.inchesToMeters(12);

  public AutoAim(Drive drive, Supplier<Double> xOffsetSupplier, Supplier<Double> yOffsetSupplier) {
    super(drive, () -> getTargetPose(drive.getPose(), xOffsetSupplier, yOffsetSupplier), false);
  }

  private static Pose2d getTargetPose(Pose2d currentPose, Supplier<Double> xOffsetSupplier, Supplier<Double> yOffsetSupplier) {
    List<Pose2d> targets = new ArrayList<>();
    for (int i = 0; i < FieldConstants.Grids.nodeRowCount; i++) {
      targets.add(
        new Pose2d(
            FieldConstants.Grids.outerX + scoringXOffset,
            FieldConstants.Grids.highTranslations[i].getY(),
            Rotation2d.fromDegrees(180)));
    }
    double xOffset = xOffsetSupplier.get() * maxXAdjustmentMeters;
    double yOffset = yOffsetSupplier.get() * maxYAdjustmentMeters;
    return currentPose.nearest(targets).transformBy(GeomUtil.translationToTransform(xOffset, yOffset));
  }
}
