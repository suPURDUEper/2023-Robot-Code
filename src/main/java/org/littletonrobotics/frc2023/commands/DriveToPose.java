// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;
import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;
import org.littletonrobotics.frc2023.util.GeomUtil;
import org.littletonrobotics.frc2023.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class DriveToPose extends CommandBase {
  private final Drive drive;
  private final Supplier<Pose2d> poseSupplier;

  private boolean running = false;
  private final ProfiledPIDController driveController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.loopPeriodSecs);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.loopPeriodSecs);
  private double driveErrorAbs;
  private double thetaErrorAbs;
  private boolean autoEnd;

  private static final LoggedTunableNumber driveKp = new LoggedTunableNumber("DriveToPose/DriveKp");
  private static final LoggedTunableNumber driveKd = new LoggedTunableNumber("DriveToPose/DriveKd");
  public static final LoggedTunableNumber thetaKp = new LoggedTunableNumber("DriveToPose/ThetaKp");
  public static final LoggedTunableNumber thetaKd = new LoggedTunableNumber("DriveToPose/ThetaKd");
  private static final LoggedTunableNumber driveMaxVelocity =
      new LoggedTunableNumber("DriveToPose/DriveMaxVelocity");
  private static final LoggedTunableNumber driveMaxAcceleration =
      new LoggedTunableNumber("DriveToPose/DriveMaxAcceleration");
  public static final LoggedTunableNumber thetaMaxVelocity =
      new LoggedTunableNumber("DriveToPose/ThetaMaxVelocity");
  public static final LoggedTunableNumber thetaMaxAcceleration =
      new LoggedTunableNumber("DriveToPose/ThetaMaxAcceleration");
  private static final LoggedTunableNumber driveTolerance =
      new LoggedTunableNumber("HoldPose/DriveTolerance");
  public static final LoggedTunableNumber thetaTolerance =
      new LoggedTunableNumber("HoldPose/ThetaTolerance");

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2023C:
      case ROBOT_SIMBOT:
        driveKp.initDefault(2.5);
        driveKd.initDefault(0.0);
        thetaKp.initDefault(7.0);
        thetaKd.initDefault(0.0);
        driveMaxVelocity.initDefault(Units.inchesToMeters(100.0));
        driveMaxAcceleration.initDefault(Units.inchesToMeters(225.0));
        thetaMaxVelocity.initDefault(Units.degreesToRadians(360.0));
        thetaMaxAcceleration.initDefault(Units.degreesToRadians(360.0));
        driveTolerance.initDefault(Units.inchesToMeters(2));
        thetaTolerance.initDefault(Units.degreesToRadians(2.0));
      default:
        break;
    }
  }

  /** Drives to the specified pose under full software control. */
  public DriveToPose(Drive drive, Pose2d pose) {
    this(drive, () -> pose);
  }

  /** Drives to the specified pose under full software control. */
  public DriveToPose(Drive drive, Supplier<Pose2d> poseSupplier) {
    this(drive, poseSupplier, true);
  }

  /** Drives to the specified pose under full software control. */
  public DriveToPose(Drive drive, Supplier<Pose2d> poseSupplier, boolean autoEnd) {
    this.drive = drive;
    this.poseSupplier = () -> AllianceFlipUtil.apply(poseSupplier.get());
    addRequirements(drive);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    this.autoEnd = autoEnd;
  }

  @Override
  public void initialize() {
    // Reset all controllers
    var currentPose = drive.getPose();
    driveController.reset(
        currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation()));
    thetaController.reset(currentPose.getRotation().getRadians());
  }

  @Override
  public void execute() {
    running = true;

    // Update from tunable numbers
    if (driveKp.hasChanged(hashCode())
        || driveKd.hasChanged(hashCode())
        || thetaKp.hasChanged(hashCode())
        || thetaKd.hasChanged(hashCode())) {
      driveController.setP(driveKp.get());
      driveController.setD(driveKd.get());
      driveController.setConstraints(
          new TrapezoidProfile.Constraints(driveMaxVelocity.get(), driveMaxAcceleration.get()));
      driveController.setTolerance(driveTolerance.get());
      thetaController.setP(thetaKp.get());
      thetaController.setD(thetaKd.get());
      thetaController.setConstraints(
          new TrapezoidProfile.Constraints(thetaMaxVelocity.get(), thetaMaxAcceleration.get()));
      thetaController.setTolerance(thetaTolerance.get());
    }

    // Get current and target pose
    var currentPose = drive.getPose();
    var targetPose = poseSupplier.get();

    // Calculate drive speed
    double currentDistance =
        currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation());
    driveErrorAbs = currentDistance;
    double driveVelocityScalar = driveController.calculate(driveErrorAbs, 0.0);
    if (driveController.atGoal()) driveVelocityScalar = 0.0;

    // Calculate theta speed
    double thetaVelocity =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    thetaErrorAbs =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
    if (thetaController.atGoal()) thetaVelocity = 0.0;

    // Command speeds
    var driveVelocity =
        new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(GeomUtil.translationToTransform(driveVelocityScalar, 0.0))
            .getTranslation();
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));

    Logger.getInstance().recordOutput("Odometry/Target Pose", targetPose);
  }

  @Override
  public boolean isFinished() {
    return atGoal() && autoEnd;
  }

  @Override
  public void end(boolean interrupted) {
    running = false;
    drive.stop();
  }

  /** Checks if the robot is stopped at the final pose. */
  public boolean atGoal() {
    return running && driveController.atGoal() && thetaController.atGoal();
  }

  /** Checks if the robot pose is within the allowed drive and theta tolerances. */
  public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
    return running
        && Math.abs(driveErrorAbs) < driveTolerance
        && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
  }
}
