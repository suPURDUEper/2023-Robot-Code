package org.supurdueper.frc2023.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
                (FieldConstants.Community.chargingStationInnerX
                            + FieldConstants.Community.chargingStationOuterX)
                        / 2
                    + .7,
                FieldConstants.Grids.lowTranslations[5].getY(),
                Rotation2d.fromDegrees(180))),
        new InstantCommand(() -> drive.setXMode(true)));
  }
}
