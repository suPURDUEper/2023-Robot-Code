package org.supurdueper.frc2023.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.littletonrobotics.frc2023.FieldConstants.StagingLocations;
import org.littletonrobotics.frc2023.commands.DriveToPose;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;
import org.supurdueper.frc2023.Constants;
import org.supurdueper.frc2023.commands.arm.ArmGoToPose;
import org.supurdueper.frc2023.commands.elevator.ElevatorGoToPose;
import org.supurdueper.frc2023.subsystems.Armavator.ArmavatorPose.ArmavatorPreset;
import org.supurdueper.frc2023.subsystems.arm.Arm;
import org.supurdueper.frc2023.subsystems.elevator.Elevator;
import org.supurdueper.frc2023.subsystems.intake.Intake;

public class ConeCubeBackupAuto extends SequentialCommandGroup {

  public ConeCubeBackupAuto(Drive drive, Elevator elevator, Arm arm, Intake intake) {

    Pose2d backup =
        new Pose2d(
            StagingLocations.translations[3].plus(new Translation2d(0, 1)),
            Rotation2d.fromDegrees(175));
    Pose2d forwardToPickup =
        new Pose2d(
            StagingLocations.translations[3].plus(new Translation2d(0, 1)),
            Rotation2d.fromDegrees(-90));

    addCommands(
        new ConeCubeAuto(drive, elevator, arm, intake),
        // Drive to middle of field
        Commands.parallel(
            new DriveToPose(drive, () -> AllianceFlipUtil.apply(backup)),
            Commands.parallel( // Wait so arm doesn't hit grid
                    new ElevatorGoToPose(elevator, ArmavatorPreset.stowed),
                    new ArmGoToPose(arm, ArmavatorPreset.stowed))
                .beforeStarting(Commands.waitSeconds(2))),
        new DriveToPose(drive, () -> AllianceFlipUtil.apply(forwardToPickup)));
  }
}
