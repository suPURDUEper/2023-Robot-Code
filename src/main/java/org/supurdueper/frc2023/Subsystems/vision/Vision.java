package org.supurdueper.frc2023.subsystems.vision;

import java.util.List;
import java.util.function.Consumer;
import org.littletonrobotics.frc2023.util.PoseEstimator.TimestampedVisionUpdate;
import org.littletonrobotics.frc2023.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class Vision extends VirtualSubsystem {

  private VisionIO visionIO;
  public VisionIOInputsAutoLogged visionInputs;
  private Consumer<List<TimestampedVisionUpdate>> visionConsumer;

  public Vision(VisionIO io, Consumer<List<TimestampedVisionUpdate>> visionConsumer) {
    visionIO = io;
    this.visionConsumer = visionConsumer;
  }

  @Override
  public void periodic() {
    visionIO.updateInputs(visionInputs);
    if (visionInputs.targetsInView > 1) {
      visionConsumer.accept(List.of(visionInputs.visionUpdate));
    }
    Logger.getInstance().processInputs("Vision", visionInputs);
  }
}
