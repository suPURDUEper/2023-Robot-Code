package org.supurdueper.frc2023.commands.auto;

import static org.supurdueper.frc2023.commands.auto.Autos.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.littletonrobotics.frc2023.FieldConstants.Grids;
import org.littletonrobotics.frc2023.FieldConstants.StagingLocations;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.util.trajectory.Waypoint;
import org.supurdueper.frc2023.Constants;
import org.supurdueper.frc2023.commands.IntakeCone;
import org.supurdueper.frc2023.commands.Score;
import org.supurdueper.frc2023.commands.armavator.ArmavatorGoToPose;
import org.supurdueper.frc2023.subsystems.Armavator.ArmavatorPose.ArmavatorPreset;
import org.supurdueper.frc2023.subsystems.arm.Arm;
import org.supurdueper.frc2023.subsystems.elevator.Elevator;
import org.supurdueper.frc2023.subsystems.intake.Intake;

public class TwoCleanAuto extends SequentialCommandGroup {

  Waypoint endPose;

  public TwoCleanAuto(Drive drive, Elevator elevator, Arm arm, Intake intake) {
    ConeHighAuto coneAuto = new ConeHighAuto(drive, elevator, arm, intake, 8);

    Waypoint pickupCube =
        waypoint(
            StagingLocations.translations[3].getX() - Constants.ROBOT_X_OFFSET / 2,
            StagingLocations.translations[3].getY(),
            Rotation2d.fromDegrees(1));

    Waypoint secondScore =
        waypoint(
            Grids.outerX + Constants.ROBOT_X_OFFSET + Units.inchesToMeters(14),
            Grids.nodeY[7],
            Rotation2d.fromDegrees(180));

    Waypoint communityTransitTurnOther =
        Autos.waypoint(communityTransitOut.getTranslation(), Rotation2d.fromDegrees(182));

    endPose = secondScore;

    addCommands(
        coneAuto,
        // Drive and intake cube
        Commands.deadline(
            new IntakeCone(intake).beforeStarting(Commands.waitSeconds(0.5)).withTimeout(3.),
            path(drive, coneAuto.getEndPose(), communityTransit, pickupCube),
            new ArmavatorGoToPose(ArmavatorPreset.intakeCone, arm, elevator)
                .beforeStarting(Commands.waitSeconds(1))),
        // Drive to grid and score cube
        Commands.parallel(
            path(drive, pickupCube, communityTransitTurnOther, communityTransit, secondScore),
            new ArmavatorGoToPose(ArmavatorPreset.coneLow, arm, elevator)
                .beforeStarting(Commands.waitSeconds(1))),
        new Score(intake).withTimeout(0.5));
  }

  public Waypoint getEndPose() {
    return endPose;
  }
}
