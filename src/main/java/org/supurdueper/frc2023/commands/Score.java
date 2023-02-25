package org.supurdueper.frc2023.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.supurdueper.frc2023.subsystems.intake.Intake;

public class Score extends CommandBase {

  private Intake intake;
  private boolean isCube;

  public Score(Intake intake, boolean isCube) {
    this.intake = intake;
    this.isCube = isCube;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.io.setCurrentLimit(40, 100, 1);
    intake.setIntakeMode(isCube ? Intake.Mode.SCORE_CUBE : Intake.Mode.SCORE_CONE);
  }

  @Override
  public void end(boolean interrupted) {
    intake.setIntakeMode(Intake.Mode.NOT_RUNNING);
  }
}
