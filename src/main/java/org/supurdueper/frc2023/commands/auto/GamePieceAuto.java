package org.supurdueper.frc2023.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.littletonrobotics.frc2023.FieldConstants.Community;
import org.littletonrobotics.frc2023.FieldConstants.Grids;
import org.littletonrobotics.frc2023.commands.DriveToPose;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.supurdueper.frc2023.commands.IntakeCone;
import org.supurdueper.frc2023.commands.Score;
import org.supurdueper.frc2023.commands.armavator.ArmavatorGoToPose;
import org.supurdueper.frc2023.commands.elevator.ResetElevatorPosition;
import org.supurdueper.frc2023.subsystems.Armavator.ArmavatorPose.ArmavatorPreset;
import org.supurdueper.frc2023.subsystems.arm.Arm;
import org.supurdueper.frc2023.subsystems.elevator.Elevator;
import org.supurdueper.frc2023.subsystems.intake.Intake;

public class GamePieceAuto extends SequentialCommandGroup {

  public GamePieceAuto(Drive drive, Elevator elevator, Arm arm, Intake intake) {
    new Rotation2d();
    drive.setPose(
        new Pose2d(
            Grids.outerX + 1,
            Grids.nodeFirstY + Grids.nodeSeparationY * 5,
            Rotation2d.fromDegrees(180)));

    addCommands(
        new ResetElevatorPosition(elevator),
        new ArmavatorGoToPose(ArmavatorPreset.highCone.getPose(), arm, elevator),
        new DriveToPose(
            drive,
            new Pose2d(
                Grids.outerX,
                Grids.nodeFirstY + Grids.nodeSeparationY * 5,
                Rotation2d.fromDegrees(180))),
        new Score(intake),
        new DriveToPose(
            drive,
            new Pose2d(
                Grids.outerX + 1,
                Community.chargingStationLeftY + .5,
                Rotation2d.fromDegrees(180))),
        new ArmavatorGoToPose(ArmavatorPreset.stowed.getPose(), arm, elevator),
        new ParallelCommandGroup(
                new DriveToPose(
                    drive,
                    new Pose2d(
                        Units.inchesToMeters(279),
                        Community.chargingStationLeftY + .5,
                        Rotation2d.fromDegrees(0))),
                new ArmavatorGoToPose(ArmavatorPreset.intakeCone.getPose(), arm, elevator),
                new IntakeCone(intake))
            .withTimeout(10));
  }
}
