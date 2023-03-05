package org.supurdueper.frc2023.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.littletonrobotics.frc2023.FieldConstants;
import org.littletonrobotics.frc2023.commands.DriveToPose;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;
import org.supurdueper.frc2023.Constants;
import org.supurdueper.frc2023.commands.IntakeCone;
import org.supurdueper.frc2023.commands.Score;
import org.supurdueper.frc2023.commands.armavator.ArmavatorGoToPose;
import org.supurdueper.frc2023.commands.drive.ResetPoseCommand;
import org.supurdueper.frc2023.commands.elevator.ResetElevatorPosition;
import org.supurdueper.frc2023.subsystems.Armavator.ArmavatorPose.ArmavatorPreset;
import org.supurdueper.frc2023.subsystems.arm.Arm;
import org.supurdueper.frc2023.subsystems.elevator.Elevator;
import org.supurdueper.frc2023.subsystems.intake.Intake;

public class ScoreConeAuto extends SequentialCommandGroup {

  public ScoreConeAuto(Drive drive, Intake intake, Arm arm, Elevator elevator, int stationIndex) {

    Pose2d start =
        AllianceFlipUtil.apply(
            new Pose2d(
                FieldConstants.Community.chargingStationInnerX - Constants.ROBOT_X_OFFSET,
                FieldConstants.Grids.lowTranslations[stationIndex].getY(),
                Rotation2d.fromDegrees(180)));

    Pose2d score =
        AllianceFlipUtil.apply(
            new Pose2d(
                FieldConstants.Grids.outerX + Constants.ROBOT_X_OFFSET,
                FieldConstants.Grids.lowTranslations[stationIndex].getY(),
                Rotation2d.fromDegrees(180)));

    Pose2d backupToRetract =
        AllianceFlipUtil.apply(
            new Pose2d(
                FieldConstants.Grids.outerX + Constants.ROBOT_X_OFFSET + Units.inchesToMeters(18),
                FieldConstants.Grids.lowTranslations[stationIndex].getY(),
                Rotation2d.fromDegrees(180)));

    addCommands( // set 0 position and score cone
        Commands.parallel(
            new ResetElevatorPosition(elevator),
            new ResetPoseCommand(drive, start),
            new InstantCommand(() -> arm.syncEncoders())),
        Commands.parallel(
            new ArmavatorGoToPose(ArmavatorPreset.highCone.getPose(), arm, elevator),
            new IntakeCone(intake)),
        new DriveToPose(drive, score),
        new Score(intake).withTimeout(1),
        new DriveToPose(drive, backupToRetract),
        new ArmavatorGoToPose(ArmavatorPreset.stowed.getPose(), arm, elevator));
  }
}
