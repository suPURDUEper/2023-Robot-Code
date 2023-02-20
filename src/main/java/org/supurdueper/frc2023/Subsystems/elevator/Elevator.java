// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.frc2023.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.frc2023.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  public final ElevatorMotorIO io;
  public final ElevatorMotorIOInputsAutoLogged inputs = new ElevatorMotorIOInputsAutoLogged();

  private static final LoggedTunableNumber elevatorKp =
      new LoggedTunableNumber("Elevator/Feedback/Kp");
  private static final LoggedTunableNumber elevatorKd =
      new LoggedTunableNumber("Elevator/Feedback/Kd");
  private static final LoggedTunableNumber elevatorKs =
      new LoggedTunableNumber("Elevator/Feedforward/Ks");
  public static final LoggedTunableNumber elevatorKg =
      new LoggedTunableNumber("Elevator/Feedforward/Kg");
  private static final LoggedTunableNumber elevatorKv =
      new LoggedTunableNumber("Elevator/Feedforward/Kv");
  private static final LoggedTunableNumber elevatorKa =
      new LoggedTunableNumber("Elevator/Feedforward/Ka");
  public static final double elevatorMaxVelocity = 1;
  public static final double elevatorMaxAcceleration = 8;
  public static final double elevatorMaxTravelM = 0.6;

  private ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(0.0, 0.0, 0.0);
  private boolean runElevatorPID = false;

  static {
    elevatorKp.initDefault(0.1);
    elevatorKd.initDefault(0);
    elevatorKs.initDefault(0.17061);
    elevatorKg.initDefault(0.17061);
    elevatorKv.initDefault(11);
    elevatorKa.initDefault(0.2752);
  }

  public Elevator(ElevatorMotorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update controllers if tunable numbers have changed
    if (elevatorKp.hasChanged(hashCode()) || elevatorKd.hasChanged(hashCode())) {
      io.setPIDGains(elevatorKp.get(), 0.0, elevatorKd.get());
    }

    if (elevatorKs.hasChanged(hashCode())
        || elevatorKg.hasChanged(hashCode())
        || elevatorKv.hasChanged(hashCode())
        || elevatorKa.hasChanged(hashCode())) {
      elevatorFeedforward =
          new ElevatorFeedforward(
              elevatorKs.get(), elevatorKg.get(), elevatorKv.get(), elevatorKa.get());
    }

    inputs.elevatorFeedforward = elevatorFeedforward.calculate(inputs.elevatorTargetVelocityMS);
    inputs.isElevatorRunningPID = runElevatorPID;
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Armavator/Motors", inputs);
  }

  public void setBrakeMode(boolean enabled) {
    io.setBrakeMode(enabled);
  }

  public double getElevatorVelocity() {
    return inputs.elevatorVelocityMS;
  }

  public double getElevatorPosition() {
    return inputs.elevatorPositionM;
  }

  public void setTargetPose(double elevatorDistance, double elevatorVelocity) {
    runElevatorPID = true;
    inputs.elevatorTargetPositionM = elevatorDistance;
    inputs.elevatorTargetVelocityMS = elevatorVelocity;
  }

  public void setVoltage(double voltage) {
    runElevatorPID = false;
    io.setVoltage(voltage);
  }

  public void resetEncoder() {
    io.resetEncoder();
  }
}
