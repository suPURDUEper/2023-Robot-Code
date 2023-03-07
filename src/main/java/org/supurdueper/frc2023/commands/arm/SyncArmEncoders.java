package org.supurdueper.frc2023.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.supurdueper.frc2023.subsystems.arm.Arm;

public class SyncArmEncoders extends InstantCommand {
  public SyncArmEncoders(Arm arm) {
    super(arm::syncEncoders, arm);
  }
}
