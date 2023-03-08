package org.supurdueper.frc2023.subsystems.vision;

import java.util.function.Consumer;
import org.littletonrobotics.frc2023.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class Vision extends VirtualSubsystem {

  private VisionIO visionIO;
  public VisionIOInputsAutoLogged visionInputs;
  private Consumer<VisionIOInputsAutoLogged> visionConsumer;

  public Vision(VisionIO io, Consumer<VisionIOInputsAutoLogged> visionConsumer) {
    visionIO = io;
    this.visionConsumer = visionConsumer;
  }

  @Override
  public void periodic() {
    Logger.getInstance().processInputs("Vision", visionInputs);
    visionIO.updateInputs(visionInputs);
    if (visionInputs.targetsInView > 0) {
      visionConsumer.accept(visionInputs);
    }
  }
}
