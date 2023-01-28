package org.supurdueper.frc2023.subsystems.Armavator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.RobotController;

public class ArmavatorIOSparkMax implements ArmavatorIO {
  private final CANSparkMax armSparkMax;
  private final CANSparkMax elevatorSparkMax;

  private final RelativeEncoder armEncoder;
  private final RelativeEncoder elevatorEncoder;

  public ArmavatorIOSparkMax() {
    elevatorSparkMax = new CANSparkMax(9, MotorType.kBrushless);
    armSparkMax = new CANSparkMax(10, MotorType.kBrushless);

    armEncoder = armSparkMax.getEncoder();
    armEncoder.setPosition(0);
    elevatorEncoder = elevatorSparkMax.getEncoder();
    elevatorEncoder.setPosition(0);
  }

  public void updateInputs(ArmavatorIOInputs inputs) {
    inputs.armPosition = armSparkMax.getEncoder().getPosition();
    inputs.elevatorPosition = elevatorSparkMax.getEncoder().getPosition();

    inputs.armAppliedVolts = armSparkMax.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.elevatorAppliedVolts =
        elevatorSparkMax.getAppliedOutput() * RobotController.getBatteryVoltage();

    inputs.armCurrentAmps = new double[] {armSparkMax.getOutputCurrent()};
    inputs.elevatorCurrentAmps = new double[] {elevatorSparkMax.getOutputCurrent()};

    inputs.armTemp = new double[] {armSparkMax.getMotorTemperature()};
    inputs.elevatorTemp = new double[] {elevatorSparkMax.getMotorTemperature()};
  }
  public void setArmVoltage(double volts) {
    armSparkMax.setVoltage(volts);
  }
  public void setElevatorVoltage(double volts) {
    elevatorSparkMax.setVoltage(volts);
  }
  public void setArmBrakeMode(boolean enable) {
    armSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
