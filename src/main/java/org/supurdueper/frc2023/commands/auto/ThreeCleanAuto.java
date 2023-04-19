package org.supurdueper.frc2023.commands.auto;

import static org.supurdueper.frc2023.commands.auto.Autos.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.littletonrobotics.frc2023.FieldConstants.Community;
import org.littletonrobotics.frc2023.FieldConstants.Grids;
import org.littletonrobotics.frc2023.FieldConstants.StagingLocations;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.util.trajectory.Waypoint;
import org.supurdueper.frc2023.Constants;
import org.supurdueper.frc2023.commands.Score;
import org.supurdueper.frc2023.commands.armavator.ArmavatorGoToPose;
import org.supurdueper.frc2023.subsystems.Armavator.ArmavatorPose.ArmavatorPreset;
import org.supurdueper.frc2023.subsystems.arm.Arm;
import org.supurdueper.frc2023.subsystems.elevator.Elevator;
import org.supurdueper.frc2023.subsystems.intake.Intake;

public class ThreeCleanAuto extends SequentialCommandGroup {

  TwoAndBackupCleanAuto coneCubeBackupAuto;

  public ThreeCleanAuto(Drive drive, Elevator elevator, Arm arm, Intake intake) {
    coneCubeBackupAuto = new TwoAndBackupCleanAuto(drive, elevator, arm, intake);

    Waypoint intermediatePoint =
        Autos.waypoint(
            StagingLocations.translations[3].getX() - Units.feetToMeters(3),
            StagingLocations.translations[3].getY(),
            Rotation2d.fromDegrees(180));

    Waypoint communityTransitIn =
        Autos.waypoint(
            Community.chargingStationOuterX + Constants.ROBOT_X_OFFSET,
            (Community.leftY - Community.chargingStationLeftY) / 2.0
                + Community.chargingStationLeftY
                - Units.feetToMeters(1),
            Rotation2d.fromDegrees(180));

    Waypoint thirdScore =
        waypoint(
            Grids.outerX + Constants.ROBOT_X_OFFSET + Units.inchesToMeters(14),
            Grids.nodeY[8] - Units.inchesToMeters(3),
            Rotation2d.fromDegrees(180));
    addCommands(
        coneCubeBackupAuto,
        Commands.parallel(
            path(
                drive,
                coneCubeBackupAuto.getEndPose(),
                intermediatePoint,
                communityTransitIn,
                communityTransit,
                thirdScore),
            new ArmavatorGoToPose(ArmavatorPreset.coneLow, arm, elevator),
            new Score(intake).beforeStarting(Commands.waitSeconds(3)))
        );
  }
}
