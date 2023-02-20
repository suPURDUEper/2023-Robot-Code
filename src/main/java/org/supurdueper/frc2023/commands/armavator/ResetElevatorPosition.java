package org.supurdueper.frc2023.commands.armavator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.supurdueper.frc2023.subsystems.elevator.Elevator;

public class ResetElevatorPosition extends CommandBase {

  Elevator elevator;

  public ResetElevatorPosition(Elevator elevator) {
    this.elevator = elevator;
    addRequirements(elevator);
  }

  @Override
  public void execute() {
    elevator.setVoltage(-2);
  }

  @Override
  public boolean isFinished() {
    return elevator.inputs.elevatorCurrentAmps[0] > 38;
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      elevator.resetEncoder();
    }
    elevator.setVoltage(0);
  }
}
