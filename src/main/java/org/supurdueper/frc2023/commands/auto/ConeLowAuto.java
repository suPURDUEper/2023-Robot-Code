package org.supurdueper.frc2023.commands.auto;

import static org.supurdueper.frc2023.commands.auto.Autos.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.littletonrobotics.frc2023.FieldConstants.Grids;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;
import org.littletonrobotics.frc2023.util.trajectory.Waypoint;
import org.supurdueper.frc2023.Constants;
import org.supurdueper.frc2023.commands.Score;
import org.supurdueper.frc2023.commands.arm.ArmGoToPose;
import org.supurdueper.frc2023.commands.elevator.ElevatorGoToPose;
import org.supurdueper.frc2023.commands.elevator.ResetElevatorPosition;
import org.supurdueper.frc2023.subsystems.Armavator.ArmavatorPose.ArmavatorPreset;
import org.supurdueper.frc2023.subsystems.arm.Arm;
import org.supurdueper.frc2023.subsystems.elevator.Elevator;
import org.supurdueper.frc2023.subsystems.intake.Intake;

public class ConeLowAuto extends SequentialCommandGroup {

  Waypoint score;

  public ConeLowAuto(Drive drive, Elevator elevator, Arm arm, Intake intake, int stationIndex) {
    Pose2d start =
        new Pose2d(
            Grids.outerX + Constants.ROBOT_X_OFFSET + Units.inchesToMeters(3),
            Grids.nodeY[stationIndex],
            Rotation2d.fromDegrees(180));

    score = Waypoint.fromHolonomicPose(start);

    addCommands(
        // Initialize robot
        Commands.parallel(
            new ResetElevatorPosition(elevator),
            Commands.runOnce(() -> drive.setPose(AllianceFlipUtil.apply(start))),
            new InstantCommand(() -> arm.syncEncoders())),

        // Drive foward and score cone
        Commands.parallel(
            new ArmGoToPose(arm, ArmavatorPreset.coneLow),
            new ElevatorGoToPose(elevator, ArmavatorPreset.coneLow)),
        new Score(intake).withTimeout(0.25));
  }

  public Waypoint getEndPose() {
    return score;
  }
}
