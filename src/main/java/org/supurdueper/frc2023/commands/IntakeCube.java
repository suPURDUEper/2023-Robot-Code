package org.supurdueper.frc2023.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.littletonrobotics.frc2023.util.LoggedTunableNumber;
import org.supurdueper.frc2023.subsystems.Intake.Intake;
import org.supurdueper.frc2023.subsystems.Intake.Intake.Mode;

public class IntakeCube extends CommandBase {
  public final Intake intake;
  private static final LoggedTunableNumber rollerAmpsLimit =
      new LoggedTunableNumber("Intake/rollerAmpsCube");

  public IntakeCube(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.setIntakeMode(Mode.INTAKE_CUBE);
  }

  @Override
  public void end(boolean interrupted) {
    intake.setIntakeMode(Mode.NOT_RUNNING);
  }

  @Override
  public boolean isFinished() {
    return intake.getRollerAmps() > rollerAmpsLimit.get();
  }
}
