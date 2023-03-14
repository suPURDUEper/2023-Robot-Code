package org.supurdueper.frc2023.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.littletonrobotics.frc2023.FieldConstants.Grids;
import org.littletonrobotics.frc2023.FieldConstants.StagingLocations;
import org.littletonrobotics.frc2023.commands.DriveToPose;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;
import org.supurdueper.frc2023.Constants;
import org.supurdueper.frc2023.commands.IntakeCube;
import org.supurdueper.frc2023.commands.Score;
import org.supurdueper.frc2023.commands.arm.ArmGoToPose;
import org.supurdueper.frc2023.commands.armavator.ArmavatorGoToPose;
import org.supurdueper.frc2023.commands.elevator.ElevatorGoToPose;
import org.supurdueper.frc2023.subsystems.Armavator.ArmavatorPose.ArmavatorPreset;
import org.supurdueper.frc2023.subsystems.arm.Arm;
import org.supurdueper.frc2023.subsystems.elevator.Elevator;
import org.supurdueper.frc2023.subsystems.intake.Intake;

public class ConeCubeAuto extends SequentialCommandGroup {

  public ConeCubeAuto(Drive drive, Elevator elevator, Arm arm, Intake intake) {
    Pose2d pickupCube =
        AllianceFlipUtil.apply(
            new Pose2d(
                StagingLocations.translations[3].plus(new Translation2d(0.5, 0.55)),
                Rotation2d.fromDegrees(-30)));

    Pose2d secondScore =
        AllianceFlipUtil.apply(
            new Pose2d(
                Grids.outerX + Constants.ROBOT_X_OFFSET + Units.inchesToMeters(20),
                Grids.nodeY[7] + Units.inchesToMeters(3),
                Rotation2d.fromDegrees(180)));

    addCommands(
        new ConeAuto(drive, elevator, arm, intake, 8),

        // Drive and intake cube
        Commands.deadline(
            new IntakeCube(intake).withTimeout(3.2),
            new DriveToPose(drive, () -> AllianceFlipUtil.apply(pickupCube)),
            Commands.parallel( // Wait to prevent arm motion while rotating
                    new ElevatorGoToPose(elevator, ArmavatorPreset.intakeCube),
                    new ArmGoToPose(arm, ArmavatorPreset.intakeCube))
                .beforeStarting(Commands.waitSeconds(0.5))),
        // Drive to grid and score cube
        Commands.parallel(
            new DriveToPose(drive, secondScore).withTimeout(3.2),
            new ArmavatorGoToPose(ArmavatorPreset.low.getPose(), arm, elevator)
                .beforeStarting(Commands.waitSeconds(1))),
        new Score(intake).withTimeout(0.5));
  }
}
