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
    ConeLowAuto coneAuto = new ConeLowAuto(drive, elevator, arm, intake, 2);

    Waypoint pickupCube =
        waypoint(
            StagingLocations.translations[0].getX() - Constants.ROBOT_X_OFFSET / 2,
            StagingLocations.translations[0].getY() + Units.inchesToMeters(6),
            Rotation2d.fromDegrees(1));

    Waypoint secondScore =
        waypoint(
            Grids.outerX + Constants.ROBOT_X_OFFSET + Units.inchesToMeters(14),
            Grids.nodeY[1],
            Rotation2d.fromDegrees(180));

    Waypoint pickupTwo =
        waypoint(
            StagingLocations.translations[1].getX() - Constants.ROBOT_X_OFFSET / 2,
            StagingLocations.translations[1].getY(),
            Rotation2d.fromDegrees(45));

    Waypoint thirdScore =
        waypoint(
            Grids.outerX + Constants.ROBOT_X_OFFSET + Units.inchesToMeters(14),
            Grids.nodeY[0] - Units.inchesToMeters(3),
            Rotation2d.fromDegrees(180));

    endPose = secondScore;

    addCommands(
        coneAuto,
        // Drive and intake cube
        Commands.deadline(
            Commands.sequence(
                new Score(intake).withTimeout(0.5),
                Commands.waitSeconds(0.75),
                new IntakeCone(intake)),
            path(drive, coneAuto.getEndPose(), communityBumpTransitFront)
                .andThen(
                    path(drive, communityBumpTransitFront, communityBumpTransitBack, pickupCube)),
            new ArmavatorGoToPose(ArmavatorPreset.intakeCone, arm, elevator)
                .beforeStarting(Commands.waitSeconds(1))),
        // Drive to grid and score cube
        Commands.parallel(
            path(drive, pickupCube, communityBumpTransitIn, communityBumpTransit, secondScore),
            new ArmavatorGoToPose(ArmavatorPreset.coneLow, arm, elevator)
                .beforeStarting(Commands.waitSeconds(1))),
        new Score(intake).withTimeout(0.5),
        // Pickup third game object
        Commands.parallel(
            new ArmavatorGoToPose(ArmavatorPreset.intakeCone, arm, elevator)
                .beforeStarting(Commands.waitSeconds(1)),
            new IntakeCone(intake),
            path(drive, secondScore, communityBumpTransitFront)
                .andThen(
                    path(drive, communityBumpTransitFront, communityBumpTransitBack, pickupTwo))),
        // Score third game object
        Commands.parallel(
            path(drive, pickupTwo, communityBumpTransitIn, communityBumpTransitBack, thirdScore),
            new ArmavatorGoToPose(ArmavatorPreset.coneLow, arm, elevator)),
        new Score(intake).withTimeout(0.5));
  }

  public Waypoint getEndPose() {
    return endPose;
  }
}
