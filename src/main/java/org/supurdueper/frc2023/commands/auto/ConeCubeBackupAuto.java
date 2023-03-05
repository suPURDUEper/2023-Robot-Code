package org.supurdueper.frc2023.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;
import org.supurdueper.frc2023.subsystems.arm.Arm;
import org.supurdueper.frc2023.subsystems.elevator.Elevator;
import org.supurdueper.frc2023.subsystems.intake.Intake;

public class ConeCubeBackupAuto extends SequentialCommandGroup {

  public ConeCubeBackupAuto(Drive drive, Elevator elevator, Arm arm, Intake intake) {
   
    Pose2d backup =
        AllianceFlipUtil.apply(
            new Pose2d()); // TODO

    addCommands(
        new ConeCubeAuto(drive, elevator, arm, intake)
    );
  }
}
