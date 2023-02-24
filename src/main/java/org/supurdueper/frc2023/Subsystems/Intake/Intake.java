package org.supurdueper.frc2023.subsystems.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private IntakeIO io;
  private Mode mode;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private static final LoggedTunableNumber rollerCubeIntakeVolts =
      new LoggedTunableNumber("Intake/CubeIntakeVolts");
  private static final LoggedTunableNumber rollerConeIntakeVolts =
      new LoggedTunableNumber("Intake/ConeIntakeVolts");
  private static final LoggedTunableNumber rollerCubeScoreVolts =
      new LoggedTunableNumber("Intake/CubeScoreVolts");
  private static final LoggedTunableNumber rollerConeScoreVolts =
      new LoggedTunableNumber("Intake/ConeScoreVolts");

  public enum Mode {
    INTAKE_CONE,
    INTAKE_CUBE,
    HOLD_CONE,
    HOLD_CUBE,
    SCORE_CUBE,
    SCORE_CONE,
    NOT_RUNNING
  }

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2023C:
        rollerCubeIntakeVolts.initDefault(4.0);
        rollerConeIntakeVolts.initDefault(-12.0);
        rollerCubeScoreVolts.initDefault(8.0);
        rollerConeScoreVolts.initDefault(8.0);
        break;
      default:
        break;
    }
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
          voltage = rollerConeIntakeVolts.get();
          break;
        case INTAKE_CUBE:
          voltage = rollerCubeIntakeVolts.get();
          break;
        case SCORE_CONE:
          voltage = rollerConeScoreVolts.get();
          break;
        case SCORE_CUBE:
          voltage = rollerCubeScoreVolts.get();
          break;
        case HOLD_CONE:
          voltage = -1;
          break;
        case HOLD_CUBE:
          voltage = 1;
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
