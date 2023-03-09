package org.supurdueper.frc2023.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.littletonrobotics.frc2023.FieldConstants.Community;
import org.littletonrobotics.frc2023.FieldConstants.Grids;
import org.littletonrobotics.frc2023.commands.DriveToPose;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;
import org.supurdueper.frc2023.Constants;
import org.supurdueper.frc2023.commands.IntakeCone;
import org.supurdueper.frc2023.commands.Score;
import org.supurdueper.frc2023.commands.arm.ArmGoToPose;
import org.supurdueper.frc2023.commands.drive.ResetPoseCommand;
import org.supurdueper.frc2023.commands.elevator.ElevatorGoToPose;
import org.supurdueper.frc2023.commands.elevator.ResetElevatorPosition;
import org.supurdueper.frc2023.subsystems.Armavator.ArmavatorPose.ArmavatorPreset;
import org.supurdueper.frc2023.subsystems.arm.Arm;
import org.supurdueper.frc2023.subsystems.elevator.Elevator;
import org.supurdueper.frc2023.subsystems.intake.Intake;

public class ConeAuto extends SequentialCommandGroup {

  public ConeAuto(Drive drive, Elevator elevator, Arm arm, Intake intake, int stationIndex) {
    Pose2d start =
        AllianceFlipUtil.apply(
            new Pose2d(
                Community.chargingStationInnerX - Constants.ROBOT_X_OFFSET,
                Grids.nodeY[stationIndex] - Units.inchesToMeters(4),
                Rotation2d.fromDegrees(180)));

    Pose2d score =
        AllianceFlipUtil.apply(
            new Pose2d(
                Grids.outerX + Constants.ROBOT_X_OFFSET,
                Grids.nodeY[stationIndex] - Units.inchesToMeters(4),
                Rotation2d.fromDegrees(180)));

    addCommands(
        // Initialize robot
        Commands.parallel(
            new ResetElevatorPosition(elevator),
            new ResetPoseCommand(drive, start),
            new InstantCommand(() -> arm.syncEncoders())),

        // Drive foward and score cone
        Commands.parallel(
            new ElevatorGoToPose(elevator, ArmavatorPreset.highCone),
            new ArmGoToPose(arm, ArmavatorPreset.highCone)
                .beforeStarting(Commands.waitSeconds(0.3)),
            new IntakeCone(intake),
            new DriveToPose(drive, score).beforeStarting(Commands.waitSeconds(0.8))),
        new Score(intake).withTimeout(0.5));
  }
}
