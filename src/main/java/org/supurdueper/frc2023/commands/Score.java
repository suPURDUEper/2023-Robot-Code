package org.supurdueper.frc2023.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.supurdueper.frc2023.subsystems.intake.Intake;

public class Score extends CommandBase {

  private Intake intake;

  public Score(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.setIntakeMode(intake.hasCube ? Intake.Mode.SCORE_CUBE : Intake.Mode.SCORE_CONE);
  }

  @Override
  public void end(boolean interrupted) {
    intake.setIntakeMode(Intake.Mode.NOT_RUNNING);
  }
}
