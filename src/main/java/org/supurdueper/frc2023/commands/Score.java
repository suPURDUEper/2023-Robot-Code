package org.supurdueper.frc2023.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.supurdueper.frc2023.subsystems.intake.Intake;

public class Score extends CommandBase {

  private Intake intake;
  private Timer timer;

  public Score(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
    timer = new Timer();
  }

  @Override
  public void initialize() {
    intake.io.setCurrentLimit(40, 100, 1);
    intake.setIntakeMode(intake.hasCube ? Intake.Mode.SCORE_CUBE : Intake.Mode.SCORE_CONE);
    timer.reset();
    timer.start();
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(1);
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    intake.setIntakeMode(Intake.Mode.NOT_RUNNING);
  }
}
