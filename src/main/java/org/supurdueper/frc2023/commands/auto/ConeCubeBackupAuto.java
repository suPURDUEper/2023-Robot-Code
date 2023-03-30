package org.supurdueper.frc2023.commands.auto;

import static org.supurdueper.frc2023.commands.auto.Autos.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.littletonrobotics.frc2023.FieldConstants.StagingLocations;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.util.trajectory.Waypoint;
import org.supurdueper.frc2023.commands.arm.ArmGoToPose;
import org.supurdueper.frc2023.commands.elevator.ElevatorGoToPose;
import org.supurdueper.frc2023.subsystems.Armavator.ArmavatorPose.ArmavatorPreset;
import org.supurdueper.frc2023.subsystems.arm.Arm;
import org.supurdueper.frc2023.subsystems.elevator.Elevator;
import org.supurdueper.frc2023.subsystems.intake.Intake;

public class ConeCubeBackupAuto extends SequentialCommandGroup {

  ConeCubeAuto coneCubeAuto;

  public ConeCubeBackupAuto(Drive drive, Elevator elevator, Arm arm, Intake intake) {

    coneCubeAuto = new ConeCubeAuto(drive, elevator, arm, intake);

    Waypoint forwardToPickup =
        waypoint(
            StagingLocations.translations[3].plus(new Translation2d(0, 1)),
            Rotation2d.fromDegrees(-90));

    addCommands(
        coneCubeAuto,
        // Drive to middle of field
        Commands.parallel(
            path(
                drive,
                coneCubeAuto.getEndPose(),
                communityTransitIn,
                communityTransitOut,
                forwardToPickup),
            Commands.parallel( // Wait so arm doesn't hit grid
                    new ElevatorGoToPose(elevator, ArmavatorPreset.stowed),
                    new ArmGoToPose(arm, ArmavatorPreset.stowed))
                .beforeStarting(Commands.waitSeconds(2))));
  }
}
