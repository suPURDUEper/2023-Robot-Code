// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.frc2023.commands.armavator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.supurdueper.frc2023.subsystems.armavator.Armavator;
import org.supurdueper.frc2023.subsystems.armavator.Armavator.ArmavatorPose;
import org.supurdueper.frc2023.subsystems.armavator.Armavator.ArmavatorPose.ArmavatorPreset;

public class ArmavatorGoToPose extends CommandBase {

  private final Armavator armavator;

  private final ArmavatorPreset target;

  private TrapezoidProfile armProfile;
  private TrapezoidProfile elevatorProfile;

  private long startTime;

  private final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(1.75, .75);

  public ArmavatorGoToPose(Armavator armavator, ArmavatorPreset target) {
    this.armavator = armavator;
    this.target = target;
    addRequirements(armavator);
  }

  @Override
  public void initialize() {
    armProfile =
        new TrapezoidProfile(
            constraints,
            target.getPose().getArmProfileState(),
            armavator.getCurrentPose().getArmProfileState());
    elevatorProfile =
        new TrapezoidProfile(
            constraints,
            target.getPose().getElevatorProfileState(),
            armavator.getCurrentPose().getElevatorProfileState());

    startTime = RobotController.getFPGATime();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    long elapsedTime = RobotController.getFPGATime() - startTime;
    double elapsedTimeSeconds = elapsedTime / 1000000.0;
    TrapezoidProfile.State armState = armProfile.calculate(elapsedTimeSeconds);
    TrapezoidProfile.State elevatorState = elevatorProfile.calculate(elapsedTimeSeconds);
    ArmavatorPose armavatorPose =
        new ArmavatorPose(
            Rotation2d.fromRadians(armState.position),
            elevatorState.position,
            armState.velocity,
            elevatorState.velocity);
    armavator.setTargetPose(armavatorPose);
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