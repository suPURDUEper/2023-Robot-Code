package org.supurdueper.frc2023.commands.auto;

import static org.supurdueper.frc2023.commands.auto.Autos.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.littletonrobotics.frc2023.FieldConstants.StagingLocations;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.util.trajectory.Waypoint;
import org.supurdueper.frc2023.commands.IntakeCone;
import org.supurdueper.frc2023.commands.armavator.ArmavatorGoToPose;
import org.supurdueper.frc2023.subsystems.Armavator.ArmavatorPose.ArmavatorPreset;
import org.supurdueper.frc2023.subsystems.arm.Arm;
import org.supurdueper.frc2023.subsystems.elevator.Elevator;
import org.supurdueper.frc2023.subsystems.intake.Intake;

public class TwoAndBackupCleanAuto extends SequentialCommandGroup {

  TwoCleanAuto coneCubeAuto;

  Waypoint forwardToPickup =
      waypoint(
          StagingLocations.translations[2].getX() - Units.inchesToMeters(6),
          StagingLocations.translations[2].getY() - Units.feetToMeters(1),
          Rotation2d.fromDegrees(-30));

  public TwoAndBackupCleanAuto(Drive drive, Elevator elevator, Arm arm, Intake intake) {

    coneCubeAuto = new TwoCleanAuto(drive, elevator, arm, intake);

    addCommands(
        coneCubeAuto,
        // Drive to middle of field
        Commands.deadline(
            new IntakeCone(intake),
            path(
                drive,
                coneCubeAuto.getEndPose(),
                communityTransit,
                communityTransitOut,
                forwardToPickup),
            new ArmavatorGoToPose(ArmavatorPreset.intakeCone, arm, elevator)
                .beforeStarting(Commands.waitSeconds(2))));
  }

  public Waypoint getEndPose() {
    return forwardToPickup;
  }
}
