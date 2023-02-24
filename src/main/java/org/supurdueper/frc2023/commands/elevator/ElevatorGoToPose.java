// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.frc2023.commands.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import org.supurdueper.frc2023.subsystems.elevator.Elevator;

public class ElevatorGoToPose extends TrapezoidProfileCommand {
  public ElevatorGoToPose(Elevator elevator, TrapezoidProfile.State target) {
    super(
        new TrapezoidProfile(
            // Limit the max acceleration and velocity
            new TrapezoidProfile.Constraints(
                Elevator.elevatorMaxVelocity, Elevator.elevatorMaxAcceleration),
            // End at desired position in meters; implicitly starts at 0
            target),
        // Pipe the profile state to the elevator
        setpointState -> elevator.setTargetPose(setpointState.position, setpointState.velocity),
        // Require the elevaotr
        elevator);
  }
}
