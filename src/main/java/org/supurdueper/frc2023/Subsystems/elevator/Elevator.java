// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.frc2023.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.frc2023.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  public static final double ELEVATOR_MAX_VELOCITY = 1;
  public static final double ELEVATOR_MAX_ACCELERATION = 10.0;

  private final ElevatorMotorIO io;
  private final ElevatorMotorIOInputsAutoLogged inputs = new ElevatorMotorIOInputsAutoLogged();

  private static final LoggedTunableNumber elevatorKp =
      new LoggedTunableNumber("Elevator/Motor/Kp");
  private static final LoggedTunableNumber elevatorKd =
      new LoggedTunableNumber("Elevator/Motor/Kd");
  private static final LoggedTunableNumber elevatorKs =
      new LoggedTunableNumber("Elevator/Motor/Ks");
  private static final LoggedTunableNumber elevatorKg =
      new LoggedTunableNumber("Elevator/Motor/Kg");
  private static final LoggedTunableNumber elevatorKv =
      new LoggedTunableNumber("Elevator/Motor/Kv");
  private static final LoggedTunableNumber elevatorKa =
      new LoggedTunableNumber("Elevator/Motor/Ka");

  private ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(0.0, 0.0, 0.0);
  private boolean runElevatorPID = false;

  static {
    elevatorKp.initDefault(10.573);
    elevatorKd.initDefault(0.021143);
    elevatorKs.initDefault(0.17061);
    elevatorKg.initDefault(0.17061);
    elevatorKv.initDefault(10.293);
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
}
