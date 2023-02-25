package org.supurdueper.frc2023.subsystems.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.frc2023.Constants;
import org.supurdueper.frc2023.subsystems.intake.IntakeIO.IntakeIOInputs;

public class Intake extends SubsystemBase {
  public IntakeIO io;
  private Mode mode;
  // private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakeIOInputs inputs = new IntakeIOInputs();

  public enum Mode {
    INTAKE_CONE,
    INTAKE_CUBE,
    HOLD_CONE,
    HOLD_CUBE,
    SCORE_CUBE,
    SCORE_CONE,
    NOT_RUNNING
  }

  /** Creates a new CubeIntake. */
  public Intake(IntakeIO io) {
    this.io = io;
    mode = Mode.NOT_RUNNING;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    // Logger.getInstance().processInputs("Intake", inputs);

    // Reset when disabled
    if (DriverStation.isDisabled()) {
      io.setRollerVoltage(0.0);
    } else {
      // Run roller
      double voltage = 0;
      switch (mode) {
        case NOT_RUNNING:
          voltage = 0.0;
          break;
        case INTAKE_CONE:
          voltage = -12;
          break;
        case INTAKE_CUBE:
          voltage = 12;
          break;
        case SCORE_CONE:
          voltage = 8;
          break;
        case SCORE_CUBE:
          voltage = -8;
          break;
        case HOLD_CONE:
          voltage = -1;
          break;
        case HOLD_CUBE:
          voltage = 0;
          break;
      }
      io.setRollerVoltage(voltage);
    }
  }

  public void setIntakeMode(Mode mode) {
    this.mode = mode;
  }

  public double getRollerAmps() {
    return io.getRollerAmps();
  }

  public Command scoreConeCommand() {
    return startEnd(() -> mode = Mode.SCORE_CONE, () -> mode = Mode.NOT_RUNNING);
  }

  public Command scoreCubeCommand() {
    return startEnd(() -> mode = Mode.SCORE_CUBE, () -> mode = Mode.NOT_RUNNING);
  }
}
