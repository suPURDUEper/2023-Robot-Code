// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.frc2023.commands.armavator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.supurdueper.frc2023.subsystems.armavator.Armavator;

public class StoweArmavator extends CommandBase {

  private final Armavator armavator;

  private final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(1.75, .75);
  private final TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private final TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  public StoweArmavator(Armavator armavator) {
    this.armavator = armavator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
