package org.supurdueper.frc2023.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.littletonrobotics.frc2023.FieldConstants;
import org.littletonrobotics.frc2023.commands.DriveToPose;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.supurdueper.frc2023.Constants;
import org.supurdueper.frc2023.commands.IntakeCube;
import org.supurdueper.frc2023.commands.Score;
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
        Commands.deadline(
                new IntakeCube(intake),
                new DriveToPose(
                    drive,
                    new Pose2d(
                        FieldConstants.StagingLocations.translations[3].plus(
                            new Translation2d(0.1, 0.2)),
                        Rotation2d.fromDegrees(-30))),
                new ArmavatorGoToPose(ArmavatorPreset.intakeCube.getPose(), arm, elevator)
        ),
        Commands.parallel(
            new DriveToPose(
                drive,
                new Pose2d(
                    FieldConstants.Grids.outerX + Constants.ROBOT_X_OFFSET + Units.feetToMeters(1),
                    FieldConstants.Grids.lowTranslations[7].getY() - Units.inchesToMeters(9),
                    Rotation2d.fromDegrees(180))),
            new WaitCommand(1)
                .andThen(new ArmavatorGoToPose(ArmavatorPreset.midCone.getPose(), arm, elevator))),
                new ArmavatorGoToPose(ArmavatorPreset.highCube.getPose(), arm, elevator),
                new Score(intake));
  }
}
