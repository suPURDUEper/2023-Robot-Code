package org.supurdueper.frc2023.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.littletonrobotics.frc2023.FieldConstants;
import org.littletonrobotics.frc2023.commands.DriveToPose;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.supurdueper.frc2023.Constants;
import org.supurdueper.frc2023.commands.IntakeCone;
import org.supurdueper.frc2023.commands.armavator.ArmavatorGoToPose;
import org.supurdueper.frc2023.subsystems.Armavator.ArmavatorPose.ArmavatorPreset;
import org.supurdueper.frc2023.subsystems.arm.Arm;
import org.supurdueper.frc2023.subsystems.elevator.Elevator;
import org.supurdueper.frc2023.subsystems.intake.Intake;

public class TwoGamePieceAuto extends SequentialCommandGroup {

  public TwoGamePieceAuto(Drive drive, Elevator elevator, Arm arm, Intake intake) {
    int startingStationIndex = 8;
    addCommands(
        new ScoreConeAuto(drive, intake, arm, elevator, startingStationIndex),
        Commands.parallel(
                new DriveToPose(
                    drive,
                    new Pose2d(
                        FieldConstants.StagingLocations.translations[3].plus(
                            new Translation2d(-0.2, 0.2)),
                        Rotation2d.fromDegrees(200))),
                new ArmavatorGoToPose(ArmavatorPreset.intakeCone.getPose(), arm, elevator),
                new IntakeCone(intake))
            .withTimeout(10),
        new DriveToPose(
            drive,
            new Pose2d(
                FieldConstants.Grids.outerX + Constants.ROBOT_X_OFFSET + Units.inchesToMeters(16),
                FieldConstants.Grids.lowTranslations[7].getY(),
                Rotation2d.fromDegrees(180))),
        Commands.parallel(
            new DriveToPose(
                drive,
                new Pose2d(
                    FieldConstants.Grids.outerX
                        + Constants.ROBOT_X_OFFSET
                        + Units.inchesToMeters(16),
                    FieldConstants.Grids.lowTranslations[6].getY(),
                    Rotation2d.fromDegrees(180))),
            new ArmavatorGoToPose(ArmavatorPreset.midCone.getPose(), arm, elevator)));
  }
}
