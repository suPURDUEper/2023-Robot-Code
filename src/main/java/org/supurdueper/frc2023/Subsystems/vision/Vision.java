package org.supurdueper.frc2023.subsystems.vision;

import java.util.function.Consumer;
import org.littletonrobotics.frc2023.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;
import org.supurdueper.frc2023.subsystems.vision.VisionIO.VisionIOInputs;

public class Vision extends VirtualSubsystem {

  private VisionIO visionIO;
  public final VisionIOInputs visionInputs = new VisionIOInputs();
  private Consumer<VisionIOInputs> visionConsumer;

  public Vision(VisionIO io, Consumer<VisionIOInputs> visionConsumer) {
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
