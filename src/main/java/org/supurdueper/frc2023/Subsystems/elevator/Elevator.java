// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.frc2023.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.supurdueper.frc2023.subsystems.elevator.ElevatorMotorIO.ElevatorMotorIOInputs;

public class Elevator extends SubsystemBase {

  public final ElevatorMotorIO io;
  // public final ElevatorMotorIOInputsAutoLogged inputs = new ElevatorMotorIOInputsAutoLogged();
  public final ElevatorMotorIOInputs inputs = new ElevatorMotorIOInputs();

  private static final double elevatorKp = 0.1;
  private static final double elevatorKd = 0;
  private static final double elevatorKs = 0.17061;
  public static final double elevatorKg = 0.17061;
  private static final double elevatorKv = 11;
  private static final double elevatorKa = 0.2752;
  public static final double elevatorMaxVelocity = 1;
  public static final double elevatorMaxAcceleration = 8;
  public static final double elevatorMaxTravelM = 0.6;

  private ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(elevatorKs, elevatorKg, elevatorKv);
  private boolean runElevatorPID = false;

  public Elevator(ElevatorMotorIO io) {
    this.io = io;
    io.setPIDGains(elevatorKp, 0.0, elevatorKd);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    inputs.elevatorFeedforward = elevatorFeedforward.calculate(inputs.elevatorTargetVelocityMS);
    inputs.isElevatorRunningPID = runElevatorPID;
    io.updateInputs(inputs);
    // Logger.getInstance().processInputs("Armavator/Motors", inputs);
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
