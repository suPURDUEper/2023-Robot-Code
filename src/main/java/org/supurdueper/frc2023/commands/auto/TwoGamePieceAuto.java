package org.supurdueper.frc2023.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.littletonrobotics.frc2023.FieldConstants;
import org.littletonrobotics.frc2023.FieldConstants.Community;
import org.littletonrobotics.frc2023.commands.DriveToPose;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;
import org.supurdueper.frc2023.Constants;
import org.supurdueper.frc2023.commands.IntakeCone;
import org.supurdueper.frc2023.commands.IntakeCube;
import org.supurdueper.frc2023.commands.Score;
import org.supurdueper.frc2023.commands.arm.ArmGoToPose;
import org.supurdueper.frc2023.commands.armavator.ArmavatorGoToPose;
import org.supurdueper.frc2023.commands.drive.ResetPoseCommand;
import org.supurdueper.frc2023.commands.elevator.ElevatorGoToPose;
import org.supurdueper.frc2023.commands.elevator.ResetElevatorPosition;
import org.supurdueper.frc2023.subsystems.Armavator.ArmavatorPose.ArmavatorPreset;
import org.supurdueper.frc2023.subsystems.arm.Arm;
import org.supurdueper.frc2023.subsystems.elevator.Elevator;
import org.supurdueper.frc2023.subsystems.intake.Intake;

public class TwoGamePieceAuto extends SequentialCommandGroup {

  public TwoGamePieceAuto(Drive drive, Elevator elevator, Arm arm, Intake intake) {
    Pose2d start =
        AllianceFlipUtil.apply(
            new Pose2d(
                FieldConstants.Community.chargingStationInnerX - Constants.ROBOT_X_OFFSET,
                FieldConstants.Grids.lowTranslations[8].getY(),
                Rotation2d.fromDegrees(180)));

    Pose2d firstScore =
        AllianceFlipUtil.apply(
            new Pose2d(
                FieldConstants.Grids.outerX + Constants.ROBOT_X_OFFSET,
                FieldConstants.Grids.lowTranslations[8].getY(),
                Rotation2d.fromDegrees(180)));

    Pose2d pickupCube =
        AllianceFlipUtil.apply(
            new Pose2d(
                FieldConstants.StagingLocations.translations[3].plus(new Translation2d(0.3, 0.4)),
                Rotation2d.fromDegrees(-30)));

    Pose2d secondScore =
        AllianceFlipUtil.apply(
            new Pose2d(
                FieldConstants.Grids.outerX + Constants.ROBOT_X_OFFSET + Units.feetToMeters(1),
                FieldConstants.Grids.lowTranslations[7].getY() - Units.inchesToMeters(8),
                Rotation2d.fromDegrees(180)));

    Pose2d inFrontOfStation =
        AllianceFlipUtil.apply(
            new Pose2d(
                FieldConstants.Community.chargingStationInnerX
                    - Constants.ROBOT_X_OFFSET
                    + Units.feetToMeters(1),
                FieldConstants.Community.chargingStationLeftY
                    - Constants.ROBOT_Y_OFFSET
                    - Units.feetToMeters(1.25),
                Rotation2d.fromDegrees(180)));

    Pose2d onStation =
        AllianceFlipUtil.apply(
            new Pose2d(
                (Community.chargingStationInnerX + Community.chargingStationOuterX) / 2
                    + Units.feetToMeters(3.5),
                FieldConstants.Community.chargingStationLeftY
                    - Constants.ROBOT_Y_OFFSET
                    - Units.feetToMeters(1.25),
                Rotation2d.fromDegrees(180)));

    addCommands(
        // Initialize robot
        Commands.parallel(
            new ResetElevatorPosition(elevator),
            new ResetPoseCommand(drive, start),
            new InstantCommand(() -> arm.syncEncoders())),

        // Drive foward and score cone
        Commands.parallel(
            new ElevatorGoToPose(elevator, ArmavatorPreset.highCone),
            new ArmGoToPose(arm, ArmavatorPreset.highCone)
                .beforeStarting(Commands.waitSeconds(0.3)),
            new IntakeCone(intake),
            new DriveToPose(drive, firstScore).beforeStarting(Commands.waitSeconds(0.8))),
        new Score(intake).withTimeout(0.5),

        // Drive and intake cube
        Commands.deadline(
            new IntakeCube(intake),
            new DriveToPose(drive, pickupCube),
            Commands.parallel( // Wait to prevent arm motion while rotating
                    new ElevatorGoToPose(elevator, ArmavatorPreset.intakeCube),
                    new ArmGoToPose(arm, ArmavatorPreset.intakeCube))
                .beforeStarting(Commands.waitSeconds(0.5))),

        // Drive to grid and score cube
        Commands.parallel(
            new DriveToPose(drive, secondScore).withTimeout(3.2),
            new ArmavatorGoToPose(ArmavatorPreset.highCube, arm, elevator)
                .beforeStarting(Commands.waitSeconds(1))),
        new Score(intake).withTimeout(0.5),

        // Drive to charging station
        Commands.parallel(
            new DriveToPose(drive, inFrontOfStation)
                .andThen(new DriveToPose(drive, onStation))
                .andThen(new InstantCommand(() -> drive.setXMode(true), drive)),
            Commands.parallel( // Wait so arm doesn't hit grid
                    new ElevatorGoToPose(elevator, ArmavatorPreset.stowed),
                    new ArmGoToPose(arm, ArmavatorPreset.stowed))
                .beforeStarting(Commands.waitSeconds(1))));
  }
}
