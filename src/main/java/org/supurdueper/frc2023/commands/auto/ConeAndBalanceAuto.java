package org.supurdueper.frc2023.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.littletonrobotics.frc2023.FieldConstants.Community;
import org.littletonrobotics.frc2023.FieldConstants.Grids;
import org.littletonrobotics.frc2023.commands.DriveToPose;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;
import org.supurdueper.frc2023.subsystems.arm.Arm;
import org.supurdueper.frc2023.subsystems.elevator.Elevator;
import org.supurdueper.frc2023.subsystems.intake.Intake;

public class ConeAndBalanceAuto extends SequentialCommandGroup {
  public ConeAndBalanceAuto(Drive drive, Intake intake, Arm arm, Elevator elevator) {

    Pose2d inFrontOfStation =
        AllianceFlipUtil.apply(
            new Pose2d(
                (Community.chargingStationOuterX + Community.chargingStationOuterX) / 2 + 1.2,
                Grids.lowTranslations[5].getY(),
                Rotation2d.fromDegrees(180)));

    Pose2d onStation =
        AllianceFlipUtil.apply(
            new Pose2d(
                (Community.chargingStationOuterX + Community.chargingStationInnerX) / 2 + .8,
                Grids.lowTranslations[5].getY(),
                Rotation2d.fromDegrees(180)));

    addCommands( // set 0 position and score cone
        new ScoreConeAuto(drive, intake, arm, elevator, 5),
        new DriveToPose(drive, inFrontOfStation),
        new DriveToPose(drive, onStation),
        new InstantCommand(() -> drive.setXMode(true)));
  }
}
