// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.frc2023.subsystems.armavator;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.frc2023.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Armavator extends SubsystemBase {

  private final ArmavatorMotorIO io;
  private final ArmavatorMotorIOInputsAutoLogged inputs = new ArmavatorMotorIOInputsAutoLogged();
  private static final LoggedTunableNumber armKp = new LoggedTunableNumber("Arm/Motor/Kp");
  private static final LoggedTunableNumber armKd = new LoggedTunableNumber("Arm/Motor/Kd");
  private static final LoggedTunableNumber armKs = new LoggedTunableNumber("Arm/Motor/Ks");
  private static final LoggedTunableNumber armKg = new LoggedTunableNumber("Arm/Motor/Kg");
  private static final LoggedTunableNumber armKv = new LoggedTunableNumber("Arm/Motor/Kv");
  private static final LoggedTunableNumber elevatorKp =
      new LoggedTunableNumber("Elevator/Motor/Kp");
  private static final LoggedTunableNumber elevatorKd =
      new LoggedTunableNumber("Elevator/Motor/Kd");
  private static final LoggedTunableNumber elevatorKs =
      new LoggedTunableNumber("Elevator/Motor/Ks");
  private static final LoggedTunableNumber elevatorKv =
      new LoggedTunableNumber("Elevator/Motor/Kv");

  private ArmFeedforward armFeedforward = new ArmFeedforward(0.0, 0.0, 0.0);
  private SimpleMotorFeedforward elevatorFeedforward = new SimpleMotorFeedforward(0.0, 0.0);

  static {
    armKp.initDefault(0.1);
    armKd.initDefault(0.0);
    armKs.initDefault(0.12349);
    armKv.initDefault(0.13477);
    elevatorKp.initDefault(0.1);
    elevatorKd.initDefault(0.0);
    elevatorKs.initDefault(0.12349);
    elevatorKv.initDefault(0.13477);
  }

  boolean isBrakeMode = true;

  public Armavator(ArmavatorMotorIO io) {
    this.io = io;

    io.setArmBrakeMode(isBrakeMode);
    io.setElevatorBrakeMode(isBrakeMode);
  }

  public static record ArmavatorPose(Rotation2d armAngle, double elevatorDistance) {
    public static enum armavatorPreset {
      stowed(new ArmavatorPose(new Rotation2d(0), 0.0)),
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
    io.updateArmavatorInputs(inputs);
    Logger.getInstance().processInputs("Armavator/Motors", inputs);

    // Update controllers if tunable numbers have changed
    if (armKp.hasChanged(hashCode()) || armKd.hasChanged(hashCode())) {
      io.setArmPIDGains(armKp.get(), 0.0, armKd.get());
    }
    if (elevatorKp.hasChanged(hashCode()) || elevatorKd.hasChanged(hashCode())) {
      io.setElevatorPIDGains(elevatorKp.get(), 0.0, elevatorKd.get());
    }
    if (armKs.hasChanged(hashCode())
        || armKv.hasChanged(hashCode())
        || armKg.hasChanged(hashCode())) {
      armFeedforward = new ArmFeedforward(armKs.get(), armKg.get(), armKv.get());
    }
    if (elevatorKs.hasChanged(hashCode()) || elevatorKv.hasChanged(hashCode())) {
      elevatorFeedforward = new SimpleMotorFeedforward(elevatorKs.get(), elevatorKv.get());
    }

    inputs.armFeedforward = armFeedforward.calculate(inputs.armPosition, inputs.armVelocity);
    inputs.elevatorFeedforward = elevatorFeedforward.calculate(inputs.elevatorVelocity);
  }

  public void Stop() {
    io.setArmVoltage(0);
    io.setElevatorVoltage(0);
  }

  public void setBrakeMode(boolean enabled) {
    io.setArmBrakeMode(enabled);
    io.setElevatorBrakeMode(enabled);
  }

  public double getArmVelocity() {
    return inputs.armVelocity;
  }

  public double getElevatorVelocity() {
    return inputs.elevatorVelocity;
  }

  public void setPose(ArmavatorPose target) {
    inputs.armTargetPosition = target.armAngle.getRadians();
    inputs.elevatorTargetPosition = target.elevatorDistance;
  }
}