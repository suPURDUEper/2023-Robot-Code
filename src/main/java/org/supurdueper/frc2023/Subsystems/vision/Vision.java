package org.supurdueper.frc2023.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

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
