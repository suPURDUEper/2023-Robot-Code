package org.supurdueper.frc2023.subsystems.intake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import org.littletonrobotics.frc2023.Constants;

public class IntakeIOTalonFX implements IntakeIO {
  private final WPI_TalonFX roller;
  private final SupplyCurrentLimitConfiguration intakeCurrentConfig;

  public IntakeIOTalonFX() {
    switch (Constants.getRobot()) {
      case ROBOT_2023C:
        roller = new WPI_TalonFX(13);
        intakeCurrentConfig = new SupplyCurrentLimitConfiguration(true, 40, 40, 0);
        break;
      default:
        throw new RuntimeException("Invalid robot for CubeIntakeIOSparkMax!");
    }
    roller.configSupplyCurrentLimit(intakeCurrentConfig);
    roller.enableVoltageCompensation(false);
    roller.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.rollerAppliedVolts = roller.getMotorOutputVoltage();
    inputs.rollerCurrentAmps = roller.getStatorCurrent();
    inputs.rollerTempCelcius = roller.getTemperature();
  }

  @Override
  public void setRollerVoltage(double voltage) {
    roller.setVoltage(voltage);
  }

  @Override
  public double getRollerAmps() {
    return roller.getStatorCurrent();
  }

  @Override
  public void setCurrentLimit(double currentLimit, double triggerAmps, double triggerTimeSeconds) {
    roller.configStatorCurrentLimit(
        new StatorCurrentLimitConfiguration(true, currentLimit, triggerAmps, triggerTimeSeconds));
  }
}
