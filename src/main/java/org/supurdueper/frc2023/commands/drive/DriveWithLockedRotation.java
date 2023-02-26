package org.supurdueper.frc2023.commands.drive;

import java.util.function.Supplier;

import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.commands.DriveToPose;
import org.littletonrobotics.frc2023.util.GeomUtil;
import org.supurdueper.frc2023.subsystems.drive.Drive;

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

public class DriveWithLockedRotation extends CommandBase {
    private static final double deadband = 0.1;
    private final Drive drive;
    private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.loopPeriodSecs);
    private Supplier<Double> thetaSupplier;
    private Supplier<Double> leftXSupplier;
    private Supplier<Double> leftYSupplier;

/** Drives to the specified pose under full software control. */
    public DriveWithLockedRotation(Drive drive, Supplier<Double> leftXSupplier, Supplier<Double> leftYSupplier,double thetaRadians) {
        this(drive, leftXSupplier, leftYSupplier, () -> thetaRadians);
    }

    /** Drives to the specified pose under full software control. */
    public DriveWithLockedRotation(Drive drive, Supplier<Double> leftXSupplier, Supplier<Double> leftYSupplier, Supplier<Double> thetaSupplier) {
        this.drive = drive;
        this.leftXSupplier = leftXSupplier;
        this.leftYSupplier = leftYSupplier;
        this.thetaSupplier = thetaSupplier;
        addRequirements(drive);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        // Reset all controllers
        var currentPose = drive.getPose();
        thetaController.reset(currentPose.getRotation().getRadians());
    }

    @Override
  public void execute() {
    // Update from tunable numbers
      thetaController.setP(DriveToPose.thetaKp.get());
      thetaController.setD(DriveToPose.thetaKd.get());
      thetaController.setConstraints(
          new TrapezoidProfile.Constraints(DriveToPose.thetaMaxVelocity.get(), DriveToPose.thetaMaxAcceleration.get()));
      thetaController.setTolerance(DriveToPose.thetaTolerance.get());

    // Get current and target pose
    var currentPose = drive.getPose();
    var targetAngle = thetaSupplier.get();

    // Get values from double suppliers
    double leftX = leftXSupplier.get();
    double leftY = leftYSupplier.get();

    // Get direction and magnitude of linear axes
    double linearMagnitude = Math.hypot(leftX, leftY);
    Rotation2d linearDirection = new Rotation2d(leftX, leftY);

    // Apply deadband
    linearMagnitude = MathUtil.applyDeadband(linearMagnitude, deadband);

    // Apply squaring
    linearMagnitude = Math.copySign(linearMagnitude * linearMagnitude, linearMagnitude);

    // Calcaulate new linear components
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(GeomUtil.translationToTransform(linearMagnitude, 0.0))
            .getTranslation();

    // Calculate theta speed
    double thetaVelocity =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), targetAngle);
    Math.abs(currentPose.getRotation().minus(Rotation2d.fromRadians(targetAngle)).getRadians());
    if (thetaController.atGoal()) thetaVelocity = 0.0;

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

    // Send to drive
    drive.runVelocity(speeds);
  }
}

