package org.supurdueper.frc2023.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.littletonrobotics.frc2023.FieldConstants;
import org.littletonrobotics.frc2023.FieldConstants.Community;
import org.littletonrobotics.frc2023.commands.DriveToPose;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;
import org.supurdueper.frc2023.Constants;
import org.supurdueper.frc2023.commands.arm.ArmGoToPose;
import org.supurdueper.frc2023.commands.elevator.ElevatorGoToPose;
import org.supurdueper.frc2023.subsystems.Armavator.ArmavatorPose.ArmavatorPreset;
import org.supurdueper.frc2023.subsystems.arm.Arm;
import org.supurdueper.frc2023.subsystems.elevator.Elevator;
import org.supurdueper.frc2023.subsystems.intake.Intake;

public class ConeCubeBalanceAuto extends SequentialCommandGroup {

  public ConeCubeBalanceAuto(Drive drive, Elevator elevator, Arm arm, Intake intake) {
    Pose2d inFrontOfStation =
        AllianceFlipUtil.apply(
            new Pose2d(
                FieldConstants.Community.chargingStationInnerX
                    - Constants.ROBOT_X_OFFSET
                    + Units.feetToMeters(1),
                FieldConstants.Community.chargingStationLeftY
                    - Constants.ROBOT_Y_OFFSET
                    - Units.feetToMeters(1.25),
                Rotation2d.fromDegrees(180)));

    Pose2d onStation =
        AllianceFlipUtil.apply(
            new Pose2d(
                Community.chargingStationCenterX + Units.feetToMeters(3.5),
                FieldConstants.Community.chargingStationLeftY
                    - Constants.ROBOT_Y_OFFSET
                    - Units.feetToMeters(1.25),
                Rotation2d.fromDegrees(180)));

    addCommands(
        new ConeCubeAuto(drive, elevator, arm, intake),

        // Drive to charging station
        Commands.parallel(
            new DriveToPose(drive, inFrontOfStation)
                .andThen(new DriveToPose(drive, onStation))
                .andThen(new InstantCommand(() -> drive.setXMode(true), drive)),
            Commands.parallel( // Wait so arm doesn't hit grid
                    new ElevatorGoToPose(elevator, ArmavatorPreset.stowed),
                    new ArmGoToPose(arm, ArmavatorPreset.stowed))
                .beforeStarting(Commands.waitSeconds(1))));
  }
}
