package org.supurdueper.frc2023.commands.auto;

import static org.supurdueper.frc2023.commands.auto.Autos.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.littletonrobotics.frc2023.FieldConstants.Community;
import org.littletonrobotics.frc2023.FieldConstants.Grids;
import org.littletonrobotics.frc2023.commands.AutoBalance;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.util.trajectory.Waypoint;
import org.supurdueper.frc2023.Constants;
import org.supurdueper.frc2023.commands.armavator.ArmavatorGoToPose;
import org.supurdueper.frc2023.subsystems.Armavator.ArmavatorPose.ArmavatorPreset;
import org.supurdueper.frc2023.subsystems.arm.Arm;
import org.supurdueper.frc2023.subsystems.elevator.Elevator;
import org.supurdueper.frc2023.subsystems.intake.Intake;

public class ConeBalanceAuto extends SequentialCommandGroup {
  public ConeBalanceAuto(Drive drive, Elevator elevator, Arm arm, Intake intake, int stationIndex) {
    ConeAuto coneAuto = new ConeAuto(drive, elevator, arm, intake, stationIndex);
    Waypoint backupToRetract =
        waypoint(
            Grids.outerX + Constants.ROBOT_X_OFFSET + Units.inchesToMeters(18),
            Grids.nodeY[stationIndex],
            Rotation2d.fromDegrees(180));

    Waypoint pastStation =
        waypoint(
            Community.chargingStationOuterX + 2,
            Grids.nodeY[stationIndex],
            Rotation2d.fromDegrees(180));

    Waypoint onStation =
        waypoint(
            Community.chargingStationCenterX + .8,
            Grids.nodeY[stationIndex],
            Rotation2d.fromDegrees(180));

    addCommands(
        coneAuto,
        Commands.deadline(
            new ArmavatorGoToPose(ArmavatorPreset.stowed, arm, elevator)
                .beforeStarting(Commands.waitSeconds(1)),
            path(drive, coneAuto.getEndPose(), backupToRetract)),
        path(drive, backupToRetract, pastStation),
        path(drive, pastStation, onStation),
        new AutoBalance(drive));
  }
}
