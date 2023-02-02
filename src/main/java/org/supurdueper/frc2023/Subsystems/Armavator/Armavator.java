// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.frc2023.subsystems.armavator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Armavator extends SubsystemBase {
  /** Creates a new Armavator. */
  ArmavatorMotor elevatorMotor;
  ArmavatorMotor armMotor;

  boolean isBrakeMode = true;

  public Armavator(ArmavatorMotorIO elevatorMotorIo, ArmavatorMotorIO armMotorIo  ) {
    elevatorMotor = new ArmavatorMotor(elevatorMotorIo, 0);
    armMotor = new ArmavatorMotor(armMotorIo, 0);

    elevatorMotor.setBrakeMode(isBrakeMode);
    armMotor.setBrakeMode(isBrakeMode);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
