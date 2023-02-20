// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.frc2023.commands.armavator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.supurdueper.frc2023.subsystems.Armavator.ArmavatorPose;
import org.supurdueper.frc2023.subsystems.arm.Arm;
import org.supurdueper.frc2023.subsystems.elevator.Elevator;

public class ArmavatorGoToPose extends CommandBase {

  private final Elevator elevator;
  private final Arm arm;
  private final ArmavatorPose target;

  private TrapezoidProfile armProfile;
  private TrapezoidProfile elevatorProfile;

  private long startTime;

  private final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(
          Elevator.elevatorMaxVelocity, Elevator.elevatorMaxAcceleration);

  public ArmavatorGoToPose(Elevator elevator, Arm arm, ArmavatorPose target) {
    this.elevator = elevator;
    this.arm = arm;
    this.target = target;
    addRequirements(elevator, arm);
  }

  @Override
  public void initialize() {
    armProfile =
        new TrapezoidProfile(
            constraints,
            target.getArmProfileState(),
            new TrapezoidProfile.State(arm.getArmPosition().getRadians(), arm.getArmVelocity()));
    elevatorProfile =
        new TrapezoidProfile(
            constraints,
            target.getElevatorProfileState(),
            new TrapezoidProfile.State(
                elevator.getElevatorPosition(), elevator.getElevatorVelocity()));

    startTime = RobotController.getFPGATime();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    long elapsedTime = RobotController.getFPGATime() - startTime;
    double elapsedTimeSeconds = elapsedTime / 1000000.0;
    TrapezoidProfile.State armState = armProfile.calculate(elapsedTimeSeconds);
    TrapezoidProfile.State elevatorState = elevatorProfile.calculate(elapsedTimeSeconds);
    arm.setTargetPose(Rotation2d.fromRadians(armState.position), armState.velocity);
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
    return armProfile.isFinished(elapsedTimeSeconds)
        && elevatorProfile.isFinished(elapsedTimeSeconds);
  }
}
