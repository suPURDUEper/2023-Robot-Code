package org.supurdueper.frc2023.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.util.SparkMaxBurnManager;

public class IntakeIOSparkMax implements IntakeIO {
  private final CANSparkMax roller;

  public IntakeIOSparkMax() {
    switch (Constants.getRobot()) {
      case ROBOT_2023C:
        roller = new CANSparkMax(13, MotorType.kBrushless);
        break;
      default:
        throw new RuntimeException("Invalid robot for CubeIntakeIOSparkMax!");
    }

    if (SparkMaxBurnManager.shouldBurn()) {
      roller.restoreFactoryDefaults();
    }

    roller.setInverted(true);
    roller.setSmartCurrentLimit(30);
    roller.enableVoltageCompensation(12.0);
    roller.setCANTimeout(0);

    if (SparkMaxBurnManager.shouldBurn()) {
      roller.burnFlash();
    }
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.rollerAppliedVolts = roller.getAppliedOutput() * roller.getBusVoltage();
    inputs.rollerCurrentAmps = roller.getOutputCurrent();
    inputs.rollerTempCelcius = roller.getMotorTemperature();
  }

  @Override
  public void setRollerVoltage(double voltage) {
    roller.setVoltage(voltage);
  }

  @Override
  public double getRollerAmps() {
    return roller.getOutputCurrent();
  }
}
