package org.supurdueper.frc2023.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.littletonrobotics.frc2023.FieldConstants.Community;
import org.littletonrobotics.frc2023.FieldConstants.Grids;
import org.littletonrobotics.frc2023.commands.DriveToPose;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;
import org.supurdueper.frc2023.Constants;
import org.supurdueper.frc2023.commands.armavator.ArmavatorGoToPose;
import org.supurdueper.frc2023.subsystems.Armavator.ArmavatorPose.ArmavatorPreset;
import org.supurdueper.frc2023.subsystems.arm.Arm;
import org.supurdueper.frc2023.subsystems.elevator.Elevator;
import org.supurdueper.frc2023.subsystems.intake.Intake;

public class ConeBalanceAuto extends SequentialCommandGroup {
  public ConeBalanceAuto(Drive drive, Elevator elevator, Arm arm, Intake intake, int stationIndex) {
    Pose2d backupToRetract =
        AllianceFlipUtil.apply(
            new Pose2d(
                Grids.outerX + Constants.ROBOT_X_OFFSET + Units.inchesToMeters(18),
                Grids.nodeY[stationIndex],
                Rotation2d.fromDegrees(180)));

    Pose2d pastStation =
        AllianceFlipUtil.apply(
            new Pose2d(
                Community.chargingStationOuterX + 2,
                Grids.nodeY[stationIndex],
                Rotation2d.fromDegrees(180)));

    Pose2d onStation =
        AllianceFlipUtil.apply(
            new Pose2d(
                Community.chargingStationCenterX + .8,
                Grids.nodeY[stationIndex],
                Rotation2d.fromDegrees(180)));

    addCommands(
        new ConeAuto(drive, elevator, arm, intake, stationIndex),
        new DriveToPose(drive, backupToRetract),
        new ArmavatorGoToPose(ArmavatorPreset.stowed, arm, elevator),
        new DriveToPose(drive, pastStation),
        new DriveToPose(drive, onStation),
        new InstantCommand(() -> drive.stopWithX(), drive));
  }
}
