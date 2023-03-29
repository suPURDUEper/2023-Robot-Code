package org.supurdueper.frc2023.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.supurdueper.frc2023.subsystems.intake.Intake;
import org.supurdueper.frc2023.subsystems.intake.Intake.Mode;

public class IntakeCone extends CommandBase {
  public final Intake intake;
  private static final double rollerAmpsLimit = 40;
  private Timer timer;

  public IntakeCone(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
    timer = new Timer();
  }

  @Override
  public void initialize() {
    intake.setIntakeMode(Mode.INTAKE_CONE);
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
      intake.hasCube = false;
      intake.setIntakeMode(Mode.HOLD_CONE);
    } else {
      intake.setIntakeMode(Mode.NOT_RUNNING);
    }
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(0.2);
  }
}
