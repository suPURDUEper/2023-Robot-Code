// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.frc2023.commands.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import org.supurdueper.frc2023.subsystems.arm.Arm;

public class ArmGoToPose extends TrapezoidProfileCommand {
  public ArmGoToPose(Arm arm, TrapezoidProfile.State target) {
    super(
        new TrapezoidProfile(
            // Limit the max acceleration and velocity
            new TrapezoidProfile.Constraints(Arm.armMaxVelocity, Arm.armMaxAcceleration),
            // End at desired position in radians
            target,
            // Start at current position
            arm.getPose()),
        // Pipe the profile state to the arm
        arm::setTargetPose,
        // Require the arm
        arm);
  }
}
