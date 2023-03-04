package org.supurdueper.frc2023.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.littletonrobotics.frc2023.FieldConstants;
import org.littletonrobotics.frc2023.FieldConstants.Community;
import org.littletonrobotics.frc2023.commands.DriveToPose;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.supurdueper.frc2023.Constants;
import org.supurdueper.frc2023.commands.IntakeCone;
import org.supurdueper.frc2023.commands.IntakeCube;
import org.supurdueper.frc2023.commands.Score;
import org.supurdueper.frc2023.commands.arm.ArmGoToPose;
import org.supurdueper.frc2023.commands.armavator.ArmavatorGoToPose;
import org.supurdueper.frc2023.commands.drive.ResetPoseCommand;
import org.supurdueper.frc2023.commands.elevator.ElevatorGoToPose;
import org.supurdueper.frc2023.commands.elevator.ResetElevatorPosition;
import org.supurdueper.frc2023.subsystems.Armavator.ArmavatorPose.ArmavatorPreset;
import org.supurdueper.frc2023.subsystems.arm.Arm;
import org.supurdueper.frc2023.subsystems.elevator.Elevator;
import org.supurdueper.frc2023.subsystems.intake.Intake;

public class TwoGamePieceAuto extends SequentialCommandGroup {

  public TwoGamePieceAuto(Drive drive, Elevator elevator, Arm arm, Intake intake) {
    int startingStationIndex = 8;
    addCommands(
        // Initialize robot
        Commands.parallel(
            new ResetElevatorPosition(elevator),
            new ResetPoseCommand(
                drive,
                new Pose2d(
                    FieldConstants.Community.chargingStationInnerX - Constants.ROBOT_X_OFFSET,
                    FieldConstants.Grids.lowTranslations[8].getY(),
                    Rotation2d.fromDegrees(180))),
            new InstantCommand(() -> arm.syncEncoders())),

        // Drive foward and score cone
        Commands.parallel(
            new ElevatorGoToPose(
                elevator, ArmavatorPreset.highCone.getPose().getElevatorProfileState()),
            new WaitCommand(0.3)
                .andThen(
                    new ArmGoToPose(arm, ArmavatorPreset.highCone.getPose().getArmProfileState())),
            new IntakeCone(intake),
            new WaitCommand(0.8)
                .andThen(
                    new DriveToPose(
                        drive,
                        new Pose2d(
                            FieldConstants.Grids.outerX + Constants.ROBOT_X_OFFSET,
                            FieldConstants.Grids.lowTranslations[8].getY(),
                            Rotation2d.fromDegrees(180))))),
        new Score(intake).withTimeout(0.5),

        // Drive and intake cube
        Commands.deadline(
            new IntakeCube(intake),
            new DriveToPose(
                drive,
                new Pose2d(
                    FieldConstants.StagingLocations.translations[3].plus(
                        new Translation2d(0.3, 0.4)),
                    Rotation2d.fromDegrees(-30))),
            new WaitCommand(0.5)
                .andThen(
                    Commands.parallel(
                        new ElevatorGoToPose(
                            elevator,
                            ArmavatorPreset.intakeCube.getPose().getElevatorProfileState()),
                        new ArmGoToPose(
                            arm, ArmavatorPreset.intakeCube.getPose().getArmProfileState())))),
        // Drive to grid and score cube
        Commands.parallel(
            new DriveToPose(
                    drive,
                    new Pose2d(
                        FieldConstants.Grids.outerX
                            + Constants.ROBOT_X_OFFSET
                            + Units.feetToMeters(1),
                        FieldConstants.Grids.lowTranslations[7].getY() - Units.inchesToMeters(8),
                        Rotation2d.fromDegrees(180)))
                .withTimeout(3.2),
            new WaitCommand(1)
                .andThen(new ArmavatorGoToPose(ArmavatorPreset.highCube.getPose(), arm, elevator))),
        new Score(intake).withTimeout(0.5),

        // Drive to charging station
        Commands.parallel(
            new DriveToPose(
                    drive,
                    new Pose2d(
                        FieldConstants.Community.chargingStationInnerX
                            - Constants.ROBOT_X_OFFSET
                            + Units.feetToMeters(1),
                        FieldConstants.Community.chargingStationLeftY
                            - Constants.ROBOT_Y_OFFSET
                            - Units.feetToMeters(1.25),
                        Rotation2d.fromDegrees(180)))
                .andThen(
                    new DriveToPose(
                        drive,
                        new Pose2d(
                            (Community.chargingStationInnerX + Community.chargingStationOuterX) / 2
                                + Units.feetToMeters(3.5),
                            FieldConstants.Community.chargingStationLeftY
                                - Constants.ROBOT_Y_OFFSET
                                - Units.feetToMeters(1.25),
                            Rotation2d.fromDegrees(180))))
                .andThen(new InstantCommand(() -> drive.setXMode(true), drive)),
            new WaitCommand(1)
                .andThen(
                    Commands.parallel(
                        new ElevatorGoToPose(
                            elevator, ArmavatorPreset.stowed.getPose().getElevatorProfileState()),
                        new ArmGoToPose(
                            arm, ArmavatorPreset.stowed.getPose().getArmProfileState())))));
  }
}
