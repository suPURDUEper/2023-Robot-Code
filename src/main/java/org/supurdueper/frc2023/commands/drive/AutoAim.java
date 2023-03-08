package org.supurdueper.frc2023.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.frc2023.FieldConstants;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;

public class AutoAim extends DriveSnapToPose {

  private static final double scoringXOffset = Units.inchesToMeters(12);

  public AutoAim(Drive drive, Supplier<Double> xOffsetSupplier, Supplier<Double> yOffsetSupplier) {
    super(drive, () -> getTargetPose(drive.getPose()), xOffsetSupplier, yOffsetSupplier);
  }

  private static Pose2d getTargetPose(Pose2d currentPose) {
    List<Pose2d> targets = new ArrayList<>();
    for (int i = 0; i < FieldConstants.Grids.nodeRowCount; i++) {
      targets.add(
          AllianceFlipUtil.apply(
              new Pose2d(
                  FieldConstants.Grids.outerX + scoringXOffset,
                  FieldConstants.Grids.highTranslations[i].getY(),
                  Rotation2d.fromRadians(0))));
    }
    return currentPose.nearest(targets);
  }
}
