// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;
import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.util.GeomUtil;
import org.littletonrobotics.frc2023.util.LoggedTunableNumber;

public class DriveWithJoysticks extends CommandBase {
  public static final LoggedTunableNumber deadband =
      new LoggedTunableNumber("DriveWithJoysticks/Deadband", 0.1);
  public static final LoggedTunableNumber minExtensionMaxLinearAcceleration =
      new LoggedTunableNumber("DriveWithJoysticks/MinExtensionMaxLinearAcceleration", 10.0);
  public static final LoggedTunableNumber fullExtensionMaxLinearAcceleration =
      new LoggedTunableNumber("DriveWithJoysticks/FullExtensionMaxLinearAcceleration", 3.0);
  public static final LoggedTunableNumber maxAngularVelocityFullExtensionPercent =
      new LoggedTunableNumber("DriveWithJoysticks/MaxAngularVelocityFullExtensionPercent", 0.75);
  public static final LoggedTunableNumber minExtensionMaxAngularVelocity =
      new LoggedTunableNumber("DriveWithJoysticks/MinExtensionMaxAngularVelocity", 9.0);
  public static final LoggedTunableNumber fullExtensionMaxAngularVelocity =
      new LoggedTunableNumber("DriveWithJoysticks/FullExtensionMaxAngularVelocity", 1.5);
  public static final LoggedTunableNumber maxAngularAcceleration =
      new LoggedTunableNumber("DriveWithJoysticks/MaxAngularAcceleration");
  public static final LoggedTunableNumber sniperModeLinearPercent =
      new LoggedTunableNumber("DriveWithJoysticks/SniperModeLinearPercent", 0.2);
  public static final LoggedTunableNumber sniperModeAngularPercent =
      new LoggedTunableNumber("DriveWithJoysticks/SniperModeAngularPercent", 0.2);
  public static final LoggedTunableNumber thetaKp =
      new LoggedTunableNumber("DriveWithJoysticks/ThetaKp");
  public static final LoggedTunableNumber thetaKd =
      new LoggedTunableNumber("DriveWithJoysticks/ThetaKd");

  private final Drive drive;
  private final Supplier<Double> leftXSupplier;
  private final Supplier<Double> leftYSupplier;
  private final Supplier<Double> rightYSupplier;
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.loopPeriodSecs);
  private final Supplier<Boolean> sniperModeSupplier;
  private final Supplier<Double> armExtensionPercentSupplier;
  private ChassisSpeeds lastSpeeds = new ChassisSpeeds();
  private Rotation2d thetaSetpoint;
  private Supplier<Boolean> rotateTo0;
  private Supplier<Boolean> rotateTo90;
  private Supplier<Boolean> rotateTo180;
  private Supplier<Boolean> rotateTo270;

  /** Creates a new DriveWithJoysticks. */
  public DriveWithJoysticks(
      Drive drive,
      Supplier<Double> leftXSupplier,
      Supplier<Double> leftYSupplier,
      Supplier<Double> rightYSupplier,
      Supplier<Boolean> sniperModeSupplier,
      Supplier<Double> armExtensionPercentSupplier,
      Supplier<Boolean> rotateTo0,
      Supplier<Boolean> rotateTo90,
      Supplier<Boolean> rotateTo180,
      Supplier<Boolean> rotateTo270) {
    addRequirements(drive);
    this.drive = drive;
    this.leftXSupplier = leftXSupplier;
    this.leftYSupplier = leftYSupplier;
    this.rightYSupplier = rightYSupplier;
    this.sniperModeSupplier = sniperModeSupplier;
    this.armExtensionPercentSupplier = armExtensionPercentSupplier;
    this.rotateTo0 = rotateTo0;
    this.rotateTo90 = rotateTo90;
    this.rotateTo180 = rotateTo180;
    this.rotateTo270 = rotateTo270;
  }

  @Override
  public void initialize() {
    lastSpeeds = new ChassisSpeeds();
    thetaSetpoint = drive.getRotation();
  }

  @Override
  public void execute() {

    /* Joystick processing */
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

    double thetaVelocity = calculateAngularVelocity(rightY);

    // Calcaulate new linear components
    double maxLinearAcceleration =
        MathUtil.interpolate(
            minExtensionMaxLinearAcceleration.get(),
            fullExtensionMaxLinearAcceleration.get(),
            armExtensionPercentSupplier.get());
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(GeomUtil.translationToTransform(linearMagnitude, 0.0))
            .getTranslation();

    // Convert to meters per second
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
            thetaVelocity);

    // Convert from field relative
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

    // Apply acceleration and velocity limits based on arm extension
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
            speeds.omegaRadiansPerSecond);
    lastSpeeds = speeds;

    // Send to drive
    drive.runVelocity(speeds);
  }

  private double calculateAngularVelocity(double rightY) {
    // Calculate velocity and acceleration constraints
    double maxAngularVelocity =
        MathUtil.interpolate(
            minExtensionMaxAngularVelocity.get(),
            fullExtensionMaxAngularVelocity.get(),
            MathUtil.clamp(
                armExtensionPercentSupplier.get() / maxAngularVelocityFullExtensionPercent.get(),
                0.0,
                1.0));

    // Update from tunable numbers
    thetaController.setP(thetaKp.get());
    thetaController.setD(thetaKd.get());
    thetaController.setConstraints(
        new TrapezoidProfile.Constraints(maxAngularVelocity, maxAngularAcceleration.get()));
    thetaController.setTolerance(0);

    // Calcualte new rotation setpoint and drive rotation velocity
    double deltaTheta = rightY * maxAngularVelocity * Constants.loopPeriodSecs;
    if (rotateTo0.get()) {
      thetaSetpoint = Rotation2d.fromDegrees(0);
    } else if (rotateTo90.get()) {
      thetaSetpoint = Rotation2d.fromDegrees(90);
    } else if (rotateTo180.get()) {
      thetaSetpoint = Rotation2d.fromDegrees(180);
    } else if (rotateTo270.get()) {
      thetaSetpoint = Rotation2d.fromDegrees(-90);
    } else {
      thetaSetpoint = thetaSetpoint.plus(Rotation2d.fromRadians(deltaTheta));
    }
    return thetaController.calculate(drive.getRotation().getRadians(), thetaSetpoint.getRadians());
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
