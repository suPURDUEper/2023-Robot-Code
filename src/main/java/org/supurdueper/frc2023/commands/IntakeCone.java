package org.supurdueper.frc2023.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.supurdueper.frc2023.RobotContainer;
import org.supurdueper.frc2023.subsystems.intake.Intake;
import org.supurdueper.frc2023.subsystems.intake.Intake.Mode;

public class IntakeCone extends CommandBase {
  public final Intake intake;
  private static final double rollerAmpsLimit = 40;

  public IntakeCone(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.setIntakeMode(Mode.INTAKE_CONE);
    intake.io.setCurrentLimit(40, rollerAmpsLimit, 1);
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      RobotContainer.hasCube = false;
      intake.setIntakeMode(Mode.HOLD_CONE);
      intake.io.setCurrentLimit(8, rollerAmpsLimit, 1);
    } else {
      intake.setIntakeMode(Mode.NOT_RUNNING);
    }
  }

  @Override
  public boolean isFinished() {
    return intake.getRollerAmps() > rollerAmpsLimit;
  }
}
