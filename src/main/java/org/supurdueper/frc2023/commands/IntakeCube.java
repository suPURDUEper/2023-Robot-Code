package org.supurdueper.frc2023.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.supurdueper.frc2023.subsystems.intake.Intake;
import org.supurdueper.frc2023.subsystems.intake.Intake.Mode;

public class IntakeCube extends CommandBase {
  public final Intake intake;
  private Timer timer;
  private static final double rollerAmpsLimit = 40;

  public IntakeCube(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
    timer = new Timer();
  }

  @Override
  public void initialize() {
    intake.setIntakeMode(Mode.INTAKE_CUBE);
    intake.io.setCurrentLimit(40, rollerAmpsLimit, 1);
    timer.reset();
  }

  @Override
  public void execute() {
    if (intake.getRollerAmps() > rollerAmpsLimit) {
      timer.start();
    } else {
      timer.restart();
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      intake.hasCube = true;
      intake.setIntakeMode(Mode.HOLD_CUBE);
      intake.io.setCurrentLimit(10, rollerAmpsLimit, 1);
    } else {
      intake.setIntakeMode(Mode.NOT_RUNNING);
    }
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(0.2);
  }
}
