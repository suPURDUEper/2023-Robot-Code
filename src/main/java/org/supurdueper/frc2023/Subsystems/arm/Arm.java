// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.frc2023.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

  public static final double armMaxVelocity = 7;
  public static final double armMaxAcceleration = 5;

  private final ArmMotorIO io;
  public final ArmMotorIOInputsAutoLogged inputs = new ArmMotorIOInputsAutoLogged();
  // public final ArmMotorIOInputs inputs = new ArmMotorIOInputs();

  private static final double armKp = 0.1;
  private static final double armKd = 0.0;
  private static final double armKs = 0.15;
  private static final double armKg = 0.25;
  private static final double armKv = 2.7;

  private ArmFeedforward armFeedforward = new ArmFeedforward(0.0, 0.0, 0.0);
  private boolean runArmPID = false;

  public Arm(ArmMotorIO io) {
    this.io = io;
    io.setPIDGains(armKp, 0.0, armKd);
    armFeedforward = new ArmFeedforward(armKs, armKg, armKv);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update controllers if tunable numbers have changed

    inputs.armFeedforward =
        armFeedforward.calculate(inputs.armTargetPositionRad - (Math.PI/2), inputs.armTargetVelocityRadS);
    inputs.isArmRunningPID = runArmPID;
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Armavator/Motors", inputs);
  }

  public void setBrakeMode(boolean enabled) {
    io.setBrakeMode(enabled);
  }

  public double getArmVelocity() {
    return inputs.armVelocityRadS;
  }

  public Rotation2d getArmPosition() {
    return Rotation2d.fromRadians(inputs.armPositionRad);
  }

  public void setTargetPose(Rotation2d armAngle, double armVelocity) {
    runArmPID = true;
    inputs.armTargetPositionRad = armAngle.getRadians();
    inputs.armTargetVelocityRadS = armVelocity;
  }

  public void setVoltage(double voltage) {
    runArmPID = false;
    io.setVoltage(voltage);
  }
}
