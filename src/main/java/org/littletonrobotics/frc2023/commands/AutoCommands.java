// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Arrays;
import java.util.List;
// import org.littletonrobotics.frc2023.AutoSelector.AutoQuestionResponse;
import org.littletonrobotics.frc2023.FieldConstants.Community;
import org.littletonrobotics.frc2023.FieldConstants.Grids;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;
import org.littletonrobotics.frc2023.util.trajectory.Waypoint;

public class AutoCommands {
  // Subsystems
  private final Drive drive;
  // private final Arm arm;
  // private final Gripper gripper;
  // private final CubeIntake cubeIntake;
  // private final Supplier<List<AutoQuestionResponse>> responses;

  // Constants
  private static final double startX = Grids.outerX + 0.5;
  private static final double throughHomeWaitSecs = 0.1;
  private static final double cubeIntakeDistance = 0.5;
  private static final double coneSweeperDistance = 1.0;
  private static final double coneSweeperBackoffDistance = 0.3;

  // Waypoints
  private final Pose2d[] startingForwards = new Pose2d[9];
  private final Pose2d[] startingBackwards = new Pose2d[9];
  private final Translation2d transitWallSide;
  private final Waypoint transitWallSideOutWaypoint;
  private final Waypoint transitWallSideInWaypoint;
  private final Translation2d transitFieldSide;
  private final Waypoint transitFieldSideOutWaypoint;
  private final Waypoint transitFieldSideInWaypoint;

  public AutoCommands(Drive drive) {
    this.drive = drive;

    for (int i = 0; i < 9; i++) {
      startingForwards[i] =
          new Pose2d(new Translation2d(startX, Grids.nodeY[i]), Rotation2d.fromDegrees(180.0));
      startingBackwards[i] =
          new Pose2d(new Translation2d(startX, Grids.nodeY[i]), new Rotation2d());
    }
    transitWallSide =
        new Translation2d(
            (Community.chargingStationInnerX + Community.chargingStationOuterX) / 2.0,
            (Community.chargingStationRightY + Community.rightY) / 2.0);
    transitWallSideInWaypoint =
        Waypoint.fromDifferentialPose(new Pose2d(transitWallSide, Rotation2d.fromDegrees(180.0)));
    transitWallSideOutWaypoint =
        Waypoint.fromDifferentialPose(new Pose2d(transitWallSide, new Rotation2d()));
    transitFieldSide =
        new Translation2d(
            (Community.chargingStationInnerX + Community.chargingStationOuterX) / 2.0,
            (Community.chargingStationLeftY + Community.leftY) / 2.0);
    transitFieldSideInWaypoint =
        Waypoint.fromDifferentialPose(new Pose2d(transitFieldSide, Rotation2d.fromDegrees(180.0)));
    transitFieldSideOutWaypoint =
        Waypoint.fromDifferentialPose(new Pose2d(transitFieldSide, new Rotation2d()));
  }

  /** Reset the odometry to the specified pose. */
  public Command reset(Pose2d pose) {
    return runOnce(() -> drive.setPose(AllianceFlipUtil.apply(pose)));
  }

  /** Returns a waypoint for a holonomic pose. */
  public Waypoint holonomic(Pose2d pose) {
    return Waypoint.fromHolonomicPose(pose);
  }

  /** Drives along the specified trajectory. */
  public Command path(Waypoint... waypoints) {
    return new DriveTrajectory(drive, Arrays.asList(waypoints), List.of());
  }

  /** Drives along the specified trajectory. */
  public Command path(List<TrajectoryConstraint> constraints, Waypoint... waypoints) {
    return new DriveTrajectory(drive, Arrays.asList(waypoints), constraints);
  }

  /** Drives to the charging station and balances on it. */
  public Command balance(Pose2d startingPosition) {
    boolean enterFront =
        startingPosition.getX()
            < (Community.chargingStationInnerX + Community.chargingStationOuterX) / 2.0;
    Pose2d position0 =
        new Pose2d(
            enterFront ? Community.chargingStationInnerX : Community.chargingStationOuterX,
            MathUtil.clamp(
                startingPosition.getY(),
                Community.chargingStationRightY + 0.8,
                Community.chargingStationLeftY - 0.8),
            Rotation2d.fromDegrees(startingPosition.getRotation().getCos() > 0.0 ? 0.0 : 180.0));
    Pose2d position1 =
        new Pose2d(
            (Community.chargingStationOuterX + Community.chargingStationInnerX) / 2.0,
            position0.getY(),
            position0.getRotation());
    return path(
            Waypoint.fromHolonomicPose(startingPosition),
            Waypoint.fromHolonomicPose(
                position0, enterFront ? new Rotation2d() : Rotation2d.fromDegrees(180.0)),
            Waypoint.fromHolonomicPose(position1))
        .andThen(new AutoBalance(drive));
  }
}
