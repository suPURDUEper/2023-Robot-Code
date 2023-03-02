package org.supurdueper.frc2023.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;

public class ResetPoseCommand extends InstantCommand {
  public ResetPoseCommand(Drive drive, Pose2d pose) {
    super(() -> drive.setPose(pose), drive);
  }

  public ResetPoseCommand(Drive drive, double x, double y, Rotation2d z) {
    new ResetPoseCommand(drive, new Pose2d(x, y, z));
  }

  public ResetPoseCommand(Drive drive, double x, double y, double z) {
    new ResetPoseCommand(drive, x, y, Rotation2d.fromRadians(z));
  }
}
