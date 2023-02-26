// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.frc2023.commands.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.supurdueper.frc2023.subsystems.elevator.Elevator;

public class ElevatorGoToPose extends CommandBase {

  private final Elevator elevator;
  private final TrapezoidProfile.State target;
  private TrapezoidProfile elevatorProfile;
  private long startTime;
  private final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(
          Elevator.elevatorMaxVelocity, Elevator.elevatorMaxAcceleration);
  ;

  public ElevatorGoToPose(Elevator elevator, TrapezoidProfile.State target) {
    this.elevator = elevator;
    this.target = target;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    elevatorProfile =
        new TrapezoidProfile(
            constraints,
            target,
            new TrapezoidProfile.State(
                elevator.getElevatorPosition(), elevator.getElevatorVelocity()));

    startTime = RobotController.getFPGATime();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    long elapsedTime = RobotController.getFPGATime() - startTime;
    double elapsedTimeSeconds = elapsedTime / 1000000.0;
    TrapezoidProfile.State elevatorState = elevatorProfile.calculate(elapsedTimeSeconds);
    elevator.setTargetPose(elevatorState.position, elevatorState.velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    long elapsedTime = RobotController.getFPGATime() - startTime;
    double elapsedTimeSeconds = elapsedTime / 1000000.0;
    return elevatorProfile.isFinished(elapsedTimeSeconds);
  }
}
