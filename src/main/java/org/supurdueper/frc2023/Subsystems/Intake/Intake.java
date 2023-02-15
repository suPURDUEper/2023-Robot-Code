package org.supurdueper.frc2023.subsystems.Intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private IntakeIO io;
  private boolean isRunning;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private static final LoggedTunableNumber rollerVolts =
      new LoggedTunableNumber("CubeIntake/RollerVolts");

  static {
    switch (Constants.getRobot()) {
      case ROBOT_SIMBOT:
        rollerVolts.initDefault(8.0);
        break;
      default:
        break;
    }
  }

  /** Creates a new CubeIntake. */
  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("CubeIntake", inputs);

    // Reset when disabled
    if (DriverStation.isDisabled()) {
      io.setRollerVoltage(0.0);
      isRunning = false;

    } else {
      // Run roller
      io.setRollerVoltage(isRunning ? rollerVolts.get() : 0.0);
    }
  }

  /** Command factory to extend and run the roller. */
  public Command runCommand() {
    return startEnd(() -> isRunning = true, () -> isRunning = false);
  }
}
