package org.supurdueper.frc2023.commands.armavator;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.supurdueper.frc2023.commands.arm.ArmGoToPose;
import org.supurdueper.frc2023.commands.elevator.ElevatorGoToPose;
import org.supurdueper.frc2023.subsystems.Armavator.ArmavatorPose;
import org.supurdueper.frc2023.subsystems.Armavator.ArmavatorPose.ArmavatorPreset;
import org.supurdueper.frc2023.subsystems.arm.Arm;
import org.supurdueper.frc2023.subsystems.elevator.Elevator;

public class ArmavatorGoToPose extends SequentialCommandGroup {

  public ArmavatorGoToPose(ArmavatorPreset preset, Arm arm, Elevator elevator) {
    this(preset.getPose(), arm, elevator);
  }

  public ArmavatorGoToPose(ArmavatorPose pose, Arm arm, Elevator elevator) {
    addCommands(
        new ElevatorGoToPose(elevator, ArmavatorPose.ELEVATOR_SAFE_TARGET)
            .unless(() -> skipElevatorSafeHeight(pose, arm, elevator)),
        Commands.parallel(
            new ArmGoToPose(arm, pose.getArmProfileState()),
            new ElevatorGoToPose(elevator, pose.getElevatorProfileState())
                .beforeStarting(Commands.waitSeconds(0.5))));
  }

  private boolean skipElevatorSafeHeight(ArmavatorPose targetPose, Arm arm, Elevator elevator) {
    return (arm.getArmPosition().getRadians() > ArmavatorPose.ARM_SAFE_ANGLE
            && targetPose.armAngle().getRadians() > ArmavatorPose.ARM_SAFE_ANGLE)
        || (elevator.getElevatorPosition() > ArmavatorPose.ELEVATOR_SAFE
            && elevator.getElevatorPosition() > ArmavatorPose.ELEVATOR_SAFE);
  }
}
