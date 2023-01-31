// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.frc2023.subsystems.armavator;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Armavator extends SubsystemBase {
  /** Creates a new Armavator. */
  CANSparkMax armSparkmax;

  CANSparkMax elevatoSparkMax;

  boolean isBrakeMode = true;

  public Armavator() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
