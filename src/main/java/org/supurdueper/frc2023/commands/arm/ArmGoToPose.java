// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.frc2023.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.supurdueper.frc2023.subsystems.arm.Arm;

public class ArmGoToPose extends CommandBase {

  private final Arm arm;
  private final TrapezoidProfile.State target;
  private TrapezoidProfile armProfile;
  private final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(Arm.armMaxVelocity, Arm.armMaxAcceleration);
  private final Timer timer = new Timer();
  ;

  public ArmGoToPose(Arm arm, TrapezoidProfile.State target) {
    this.arm = arm;
    this.target = target;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    armProfile =
        new TrapezoidProfile(
            constraints,
            target,
            new TrapezoidProfile.State(arm.getArmPosition().getRadians(), arm.getArmVelocity()));
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double elapsedTimeSeconds = timer.get();
    TrapezoidProfile.State armState = armProfile.calculate(elapsedTimeSeconds);
    arm.setTargetPose(Rotation2d.fromRadians(armState.position), armState.velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(armProfile.totalTime());
  }
}
