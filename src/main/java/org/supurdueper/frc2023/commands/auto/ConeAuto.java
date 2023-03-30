package org.supurdueper.frc2023.commands.auto;

import static org.supurdueper.frc2023.commands.auto.Autos.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.littletonrobotics.frc2023.FieldConstants.Community;
import org.littletonrobotics.frc2023.FieldConstants.Grids;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;
import org.littletonrobotics.frc2023.util.trajectory.Waypoint;
import org.supurdueper.frc2023.Constants;
import org.supurdueper.frc2023.commands.IntakeCone;
import org.supurdueper.frc2023.commands.Score;
import org.supurdueper.frc2023.commands.arm.ArmGoToPose;
import org.supurdueper.frc2023.commands.elevator.ElevatorGoToPose;
import org.supurdueper.frc2023.commands.elevator.ResetElevatorPosition;
import org.supurdueper.frc2023.subsystems.Armavator.ArmavatorPose.ArmavatorPreset;
import org.supurdueper.frc2023.subsystems.arm.Arm;
import org.supurdueper.frc2023.subsystems.elevator.Elevator;
import org.supurdueper.frc2023.subsystems.intake.Intake;

public class ConeAuto extends SequentialCommandGroup {

  Waypoint score;

  public ConeAuto(Drive drive, Elevator elevator, Arm arm, Intake intake, int stationIndex) {
    Pose2d start =
        new Pose2d(
            Community.chargingStationInnerX - Constants.ROBOT_X_OFFSET,
            Grids.nodeY[stationIndex],
            Rotation2d.fromDegrees(180));

    score =
        waypoint(
            Grids.outerX + Constants.ROBOT_X_OFFSET - Units.inchesToMeters(2),
            Grids.nodeY[stationIndex],
            Rotation2d.fromDegrees(180));

    addCommands(
        // Initialize robot
        Commands.parallel(
            new ResetElevatorPosition(elevator),
            Commands.runOnce(() -> drive.setPose(AllianceFlipUtil.apply(start))),
            new InstantCommand(() -> arm.syncEncoders())),

        // Drive foward and score cone
        Commands.parallel(
                new ElevatorGoToPose(elevator, ArmavatorPreset.highCone),
                new ArmGoToPose(arm, ArmavatorPreset.coneLow)
                    .beforeStarting(Commands.waitSeconds(0.3)),
                new IntakeCone(intake).withTimeout(1),
                path(drive, Waypoint.fromHolonomicPose(start), score)
                    .beforeStarting(Commands.waitSeconds(0.8)))
            // new DriveToPose(drive, score).beforeStarting(Commands.waitSeconds(0.8)))
            .withTimeout(4),
        Commands.runOnce(() -> drive.stop(), drive),
        new Score(intake).withTimeout(0.5));
  }

  public Waypoint getEndPose() {
    return score;
  }
}
