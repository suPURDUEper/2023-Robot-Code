package org.supurdueper.frc2023.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Arrays;
import java.util.List;
import org.littletonrobotics.frc2023.FieldConstants.Community;
import org.littletonrobotics.frc2023.commands.DriveTrajectory;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.util.trajectory.Waypoint;
import org.supurdueper.frc2023.Constants;

public class Autos {

  public static Waypoint communityTransit =
      Autos.waypoint(
          Community.chargingStationCenterX,
          (Community.leftY - Community.chargingStationLeftY) / 2.0 + Community.chargingStationLeftY,
          Rotation2d.fromDegrees(180));

  public static Waypoint communityTransitOut =
      Autos.waypoint(
          Community.chargingStationOuterX + Constants.ROBOT_X_OFFSET,
          (Community.leftY - Community.chargingStationLeftY) / 2.0
              + Community.chargingStationLeftY);

  public static Command path(Drive drive, Waypoint... waypoints) {
    return new DriveTrajectory(drive, Arrays.asList(waypoints), List.of());
  }

  /** Returns a waypoint for a holonomic pose. */
  public static Waypoint waypoint(double x, double y, Rotation2d rot) {
    return Waypoint.fromHolonomicPose(new Pose2d(x, y, rot));
  }

  /** Returns a waypoint for a holonomic pose. */
  public static Waypoint waypoint(Translation2d translation, Rotation2d rot) {
    return Waypoint.fromHolonomicPose(new Pose2d(translation, rot));
  }

  /** Returns a waypoint for a holonomic pose. */
  public static Waypoint waypoint(Translation2d translation) {
    return new Waypoint(translation);
  }

  /** Returns a waypoint for a holonomic pose. */
  public static Waypoint waypoint(double x, double y) {
    return new Waypoint(new Translation2d(x, y));
  }
}
