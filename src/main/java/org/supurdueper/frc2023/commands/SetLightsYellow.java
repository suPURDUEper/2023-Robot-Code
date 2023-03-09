package org.supurdueper.frc2023.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.supurdueper.frc2023.subsystems.Lights;

public class SetLightsYellow extends CommandBase {
  public final Lights lights;

  public SetLightsYellow(Lights lights) {
    this.lights = lights;
    addRequirements(lights);
  }

  @Override
  public void initialize() {
    lights.setLightsYellow();
  }
}
