package org.supurdueper.frc2023.subsystems.vision;

import org.littletonrobotics.frc2023.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class Vision extends VirtualSubsystem {

  private VisionIO visionIO;
  private VisionIOInputsAutoLogged visionInputs;

  public Vision(VisionIO io) {
    visionIO = io;
  }

  @Override
  public void periodic() {
    visionIO.updateInputs(visionInputs);
    Logger.getInstance().processInputs("Vision", visionInputs);
  }
}
