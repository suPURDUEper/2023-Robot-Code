package org.supurdueper.frc2023.commands.auto;

import org.littletonrobotics.frc2023.subsystems.drive.Drive;

import java.lang.reflect.Array;

import org.littletonrobotics.frc2023.FieldConstants;
import org.supurdueper.frc2023.subsystems.arm.Arm;
import org.supurdueper.frc2023.subsystems.elevator.Elevator;
import org.supurdueper.frc2023.subsystems.intake.Intake;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class GamePieceAuto extends SequentialCommandGroup {

    public GamePieceAuto(Drive drive, Elevator elevator , Arm arm, Intake intake) {
        new Rotation2d();
        drive.setPose(new Pose2d((Translation2d)Array.get(FieldConstants.Grids.complexLowTranslations, 6), Rotation2d.fromDegrees(180)));
        
        addCommands(null);
    }
    
}
