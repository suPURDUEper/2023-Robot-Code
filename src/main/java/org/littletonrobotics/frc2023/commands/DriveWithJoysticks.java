// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;
import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;
import org.littletonrobotics.frc2023.util.GeomUtil;
import org.littletonrobotics.frc2023.util.LoggedTunableNumber;

public class DriveWithJoysticks extends CommandBase {
  public static final LoggedTunableNumber deadband =
      new LoggedTunableNumber("DriveWithJoysticks/Deadband", 0.1);
  public static final LoggedTunableNumber minExtensionMaxLinearAcceleration =
      new LoggedTunableNumber("DriveWithJoysticks/MinExtensionMaxLinearAcceleration", 20.0);
  public static final LoggedTunableNumber fullExtensionMaxLinearAcceleration =
      new LoggedTunableNumber("DriveWithJoysticks/FullExtensionMaxLinearAcceleration", 3.0);
  public static final LoggedTunableNumber maxAngularVelocityFullExtensionPercent =
      new LoggedTunableNumber("DriveWithJoysticks/MaxAngularVelocityFullExtensionPercent", 0.3);
  public static final LoggedTunableNumber minExtensionMaxAngularVelocity =
      new LoggedTunableNumber("DriveWithJoysticks/MinExtensionMaxAngularVelocity", 9.0);
  public static final LoggedTunableNumber fullExtensionMaxAngularVelocity =
      new LoggedTunableNumber("DriveWithJoysticks/FullExtensionMaxAngularVelocity", 1.5);
  public static final LoggedTunableNumber sniperModeLinearPercent =
      new LoggedTunableNumber("DriveWithJoysticks/SniperModeLinearPercent", 0.275);
  public static final LoggedTunableNumber sniperModeAngularPercent =
      new LoggedTunableNumber("DriveWithJoysticks/SniperModeAngularPercent", 0.2);

  private final Drive drive;
  private final Supplier<Double> leftXSupplier;
  private final Supplier<Double> leftYSupplier;
  private final Supplier<Double> rightYSupplier;
  private final Supplier<Boolean> sniperModeSupplier;
  private final Supplier<Boolean> robotRelativeOverride;
  private final Supplier<Double> armExtensionPercentSupplier;
  private final Supplier<Boolean> lockTo0;
  private final Supplier<Boolean> lockTo90;
  private final Supplier<Boolean> lockTo180;
  private final Supplier<Boolean> lockTo270;
  private ChassisSpeeds lastSpeeds = new ChassisSpeeds();

  private final PIDController thetaController = new PIDController(8.0, 0.0, 0.0);
  Rotation2d lastCommandedRotation;
  double lastRightY;

  /** Creates a new DriveWithJoysticks. */
  public DriveWithJoysticks(
      Drive drive,
      Supplier<Double> leftXSupplier,
      Supplier<Double> leftYSupplier,
      Supplier<Double> rightYSupplier,
      Supplier<Boolean> sniperModeSupplier,
      Supplier<Boolean> robotRelativeOverride,
      Supplier<Double> armExtensionPercentSupplier,
      Supplier<Boolean> lockTo0,
      Supplier<Boolean> lockTo90,
      Supplier<Boolean> lockTo180,
      Supplier<Boolean> lockTo270) {
    addRequirements(drive);
    this.drive = drive;
    this.leftXSupplier = leftXSupplier;
    this.leftYSupplier = leftYSupplier;
    this.rightYSupplier = rightYSupplier;
    this.sniperModeSupplier = sniperModeSupplier;
    this.robotRelativeOverride = robotRelativeOverride;
    this.armExtensionPercentSupplier = armExtensionPercentSupplier;
    this.lockTo0 = lockTo0;
    this.lockTo90 = lockTo90;
    this.lockTo180 = lockTo180;
    this.lockTo270 = lockTo270;
  }

  @Override
  public void initialize() {
    lastSpeeds = new ChassisSpeeds();
    lastRightY = 0;
    lastCommandedRotation = drive.getPose().getRotation();
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void execute() {
    // Get values from double suppliers
    double leftX = leftXSupplier.get();
    double leftY = leftYSupplier.get();
    double rightY = rightYSupplier.get();

    // Get direction and magnitude of linear axes
    double linearMagnitude = Math.hypot(leftX, leftY);
    Rotation2d linearDirection = new Rotation2d(leftX, leftY);

    // Apply deadband
    linearMagnitude = MathUtil.applyDeadband(linearMagnitude, deadband.get());
    rightY = MathUtil.applyDeadband(rightY, deadband.get());

    // Apply squaring
    linearMagnitude = Math.copySign(linearMagnitude * linearMagnitude, linearMagnitude);
    rightY = Math.copySign(rightY * rightY, rightY);

    // Apply speed limits
    if (sniperModeSupplier.get()) {
      linearMagnitude *= sniperModeLinearPercent.get();
      rightY *= sniperModeAngularPercent.get();
    }

    // Calcaulate new linear components
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(GeomUtil.translationToTransform(linearMagnitude, 0.0))
            .getTranslation();

    // Lock rotation when not using rotation stick
    if (lastRightY != 0 && rightY == 0) {
      lastCommandedRotation = drive.getPose().getRotation();
      thetaController.reset();
    }
    double thetaVelocityRadPerS = rightY * minExtensionMaxAngularVelocity.get();
    if (lockTo0.get()) {
      lastCommandedRotation = AllianceFlipUtil.apply(Rotation2d.fromDegrees(0));
    } else if (lockTo90.get()) {
      lastCommandedRotation = AllianceFlipUtil.apply(Rotation2d.fromDegrees(90));
    } else if (lockTo180.get()) {
      lastCommandedRotation = AllianceFlipUtil.apply(Rotation2d.fromDegrees(180));
    } else if (lockTo270.get()) {
      lastCommandedRotation = AllianceFlipUtil.apply(Rotation2d.fromDegrees(-90));
    }
    if ((rightY == 0.0 && linearVelocity.getNorm() > 1.0)
        || lockTo0.get()
        || lockTo90.get()
        || lockTo180.get()
        || lockTo270.get()) {
      thetaVelocityRadPerS =
          thetaController.calculate(
              drive.getPose().getRotation().getRadians(), lastCommandedRotation.getRadians());
    }
    lastRightY = rightY;

    // Convert to meters per second
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
            thetaVelocityRadPerS);

    // Convert from field relative
    if (!robotRelativeOverride.get()) {
      var driveRotation = drive.getRotation();
      if (DriverStation.getAlliance() == Alliance.Red) {
        driveRotation = driveRotation.plus(new Rotation2d(Math.PI));
      }
      speeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              speeds.vxMetersPerSecond,
              speeds.vyMetersPerSecond,
              speeds.omegaRadiansPerSecond,
              driveRotation);
    }

    // Apply acceleration and velocity limits based on arm extension
    double maxLinearAcceleration =
        MathUtil.interpolate(
            minExtensionMaxLinearAcceleration.get(),
            fullExtensionMaxLinearAcceleration.get(),
            armExtensionPercentSupplier.get());
    double maxAngularVelocity =
        MathUtil.interpolate(
            minExtensionMaxAngularVelocity.get(),
            fullExtensionMaxAngularVelocity.get(),
            MathUtil.clamp(
                armExtensionPercentSupplier.get() / maxAngularVelocityFullExtensionPercent.get(),
                0.0,
                1.0));
    speeds =
        new ChassisSpeeds(
            MathUtil.clamp(
                speeds.vxMetersPerSecond,
                lastSpeeds.vxMetersPerSecond - maxLinearAcceleration * Constants.loopPeriodSecs,
                lastSpeeds.vxMetersPerSecond + maxLinearAcceleration * Constants.loopPeriodSecs),
            MathUtil.clamp(
                speeds.vyMetersPerSecond,
                lastSpeeds.vyMetersPerSecond - maxLinearAcceleration * Constants.loopPeriodSecs,
                lastSpeeds.vyMetersPerSecond + maxLinearAcceleration * Constants.loopPeriodSecs),
            MathUtil.clamp(speeds.omegaRadiansPerSecond, -maxAngularVelocity, maxAngularVelocity));
    lastSpeeds = speeds;

    // Send to drive
    drive.runVelocity(speeds);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
