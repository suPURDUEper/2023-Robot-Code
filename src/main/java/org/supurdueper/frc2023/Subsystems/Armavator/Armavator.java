// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.frc2023.subsystems.armavator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Armavator extends SubsystemBase {
  /** Creates a new Armavator. */
  ArmavatorMotor armavatorMotor;

  private final ArmavatorMotorIO io;
  private final ArmavatorMotorIOInputsAutoLogged inputs = new ArmavatorMotorIOInputsAutoLogged();

  boolean isBrakeMode = true;

  public Armavator(ArmavatorMotorIO io) {
    this.io = io;

    armavatorMotor.setBrakeMode(isBrakeMode);
  }

  public static record ArmavatorPose(Rotation2d armAngle, double elevatorDistance) {
    public static enum armavatorPreset {
      stowed(null),
      intake(new ArmavatorPose(new Rotation2d(Math.PI / 4), 0.5)),
      low(new ArmavatorPose(new Rotation2d(Math.PI / 4), 1.0)),
      midCone(new ArmavatorPose(new Rotation2d(Math.PI / 4), 0.5)),
      midCube(new ArmavatorPose(new Rotation2d(Math.PI / 4), 0.5)),
      highCone(new ArmavatorPose(new Rotation2d(Math.PI / 4), 0.5)),
      highCube(new ArmavatorPose(new Rotation2d(Math.PI / 4), 0.5));

      private ArmavatorPose pose;

      private armavatorPreset(ArmavatorPose pose) {
        this.pose = pose;
      }

      public ArmavatorPose getPose() {
        return pose;
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    armavatorMotor.periodic();
  }
}
