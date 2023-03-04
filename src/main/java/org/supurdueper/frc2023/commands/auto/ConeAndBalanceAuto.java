package org.supurdueper.frc2023.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import org.littletonrobotics.frc2023.FieldConstants;
import org.littletonrobotics.frc2023.commands.DriveToPose;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.supurdueper.frc2023.subsystems.arm.Arm;
import org.supurdueper.frc2023.subsystems.elevator.Elevator;
import org.supurdueper.frc2023.subsystems.intake.Intake;

public class ConeAndBalanceAuto extends SequentialCommandGroup {
  public ConeAndBalanceAuto(Drive drive, Intake intake, Arm arm, Elevator elevator) {
    addCommands( // set 0 position and score cone
        new ScoreConeAuto(drive, intake, arm, elevator, 5),
        new DriveToPose(
            drive,
            new Pose2d(
                (FieldConstants.Community.chargingStationOuterX
                            + FieldConstants.Community.chargingStationOuterX)
                        / 2
                    + 1.2,
                FieldConstants.Grids.lowTranslations[5].getY(),
                Rotation2d.fromDegrees(180))),
        new DriveToPose(
            drive,
            new Pose2d(
                (FieldConstants.Community.chargingStationOuterX
                            + FieldConstants.Community.chargingStationInnerX)
                        / 2
                    +.8,
                FieldConstants.Grids.lowTranslations[5].getY(),
                Rotation2d.fromDegrees(180))),
        new InstantCommand(() -> drive.setXMode(true))
        .andThen(new WaitCommand(1))
        .andThen(new InstantCommand(() -> drive.setXMode(false))));
  }
}
