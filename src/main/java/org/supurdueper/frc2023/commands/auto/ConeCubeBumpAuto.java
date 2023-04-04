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

public class ConeCubeBumpAuto extends SequentialCommandGroup {

  Waypoint endPose;

  public ConeCubeBumpAuto(Drive drive, Elevator elevator, Arm arm, Intake intake) {
    ConeAuto coneAuto = new ConeAuto(drive, elevator, arm, intake, 0);

    Waypoint pickupCube =
        waypoint(
            StagingLocations.translations[0].getX() - Constants.ROBOT_X_OFFSET / 2,
            StagingLocations.translations[0].getY(),
            Rotation2d.fromDegrees(1));

    Waypoint secondScore =
        waypoint(
            Grids.outerX + Constants.ROBOT_X_OFFSET + Units.inchesToMeters(6),
            Grids.nodeY[1] + Units.inchesToMeters(2),
            Rotation2d.fromDegrees(180));

    endPose = secondScore;

    addCommands(
        coneAuto,
        // Drive and intake cube
        Commands.deadline(
            new IntakeCone(intake), // .withTimeout(3.7),
            path(drive, coneAuto.getEndPose(), communityBumpTransit).andThen(
                path(drive, communityBumpTransit, pickupCube)
            ),
            new ArmavatorGoToPose(ArmavatorPreset.intakeCone, arm, elevator)
                .beforeStarting(Commands.waitSeconds(1))),
        // Drive to grid and score cube
        Commands.parallel(
            path(drive, pickupCube, communityBumpTransitOut, communityBumpTransit, secondScore),
            new ArmavatorGoToPose(ArmavatorPreset.coneLow, arm, elevator)
                .beforeStarting(Commands.waitSeconds(1))),
        new Score(intake).withTimeout(0.5));
  }

  public Waypoint getEndPose() {
    return endPose;
  }
}
