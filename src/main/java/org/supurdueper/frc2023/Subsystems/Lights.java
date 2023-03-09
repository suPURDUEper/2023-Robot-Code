package org.supurdueper.frc2023.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
  private final Spark lights = new Spark(5);

  public static final double purple = .91;
  public static final double yellow = .69;
  public static final double gold = .67;
  public static final double black = .99;

  public void setLightsPurple() {
    lights.set(purple);
  }

  public void setLightsYellow() {
    lights.set(yellow);
  }
}
