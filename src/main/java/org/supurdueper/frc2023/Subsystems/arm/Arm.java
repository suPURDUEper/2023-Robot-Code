// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.frc2023.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.frc2023.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

  public static final double ELEVATOR_MAX_VELOCITY = 1;
  public static final double ELEVATOR_MAX_ACCELERATION = 10.0;

  private final ArmMotorIO io;
  private final ArmMotorIOInputsAutoLogged inputs = new ArmMotorIOInputsAutoLogged();
  private static final LoggedTunableNumber armKp = new LoggedTunableNumber("Arm/Motor/Kp");
  private static final LoggedTunableNumber armKd = new LoggedTunableNumber("Arm/Motor/Kd");
  private static final LoggedTunableNumber armKs = new LoggedTunableNumber("Arm/Motor/Ks");
  private static final LoggedTunableNumber armKg = new LoggedTunableNumber("Arm/Motor/Kg");
  private static final LoggedTunableNumber armKv = new LoggedTunableNumber("Arm/Motor/Kv");

  private ArmFeedforward armFeedforward = new ArmFeedforward(0.0, 0.0, 0.0);
  private boolean runArmPID = false;

  static {
    armKp.initDefault(0.1);
    armKd.initDefault(0.0);
    armKs.initDefault(0.12349);
    armKv.initDefault(0.13477);
  }

  public Arm(ArmMotorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update controllers if tunable numbers have changed
    if (armKp.hasChanged(hashCode()) || armKd.hasChanged(hashCode())) {
      io.setPIDGains(armKp.get(), 0.0, armKd.get());
    }

    if (armKs.hasChanged(hashCode())
        || armKv.hasChanged(hashCode())
        || armKg.hasChanged(hashCode())) {
      armFeedforward = new ArmFeedforward(armKs.get(), armKg.get(), armKv.get());
    }

    inputs.armFeedforward =
        armFeedforward.calculate(inputs.armTargetPositionRad, inputs.armTargetVelocityRadS);
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
