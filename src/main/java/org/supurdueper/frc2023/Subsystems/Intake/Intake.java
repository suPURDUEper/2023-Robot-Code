package org.supurdueper.frc2023.subsystems.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  public IntakeIO io;
  private Mode mode;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  public boolean hasCube;

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
    Logger.getInstance().processInputs("Intake", inputs);
    Logger.getInstance().recordOutput("Intake/hasCube", hasCube);
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
          io.setCurrentLimit(200, 200, 1);
          voltage = -12;
          break;
        case INTAKE_CUBE:
          io.setCurrentLimit(200, 200, 1);
          voltage = 12;
          break;
        case SCORE_CONE:
          io.setCurrentLimit(40, 40, 1);
          voltage = 8;
          break;
        case SCORE_CUBE:
          io.setCurrentLimit(40, 40, 1);
          voltage = -8;
          break;
        case HOLD_CONE:
          io.setCurrentLimit(17, 17, 1);
          voltage = -10;
          break;
        case HOLD_CUBE:
          io.setCurrentLimit(25, 25, 1);
          voltage = 10;
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

  public boolean hasCube() {
    return hasCube;
  }
}
