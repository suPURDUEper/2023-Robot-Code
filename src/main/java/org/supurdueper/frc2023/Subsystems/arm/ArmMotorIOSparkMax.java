package org.supurdueper.frc2023.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.frc2023.util.SparkMaxBurnManager;

public class ArmMotorIOSparkMax implements ArmMotorIO {
  private final CANSparkMax armSparkMax;
  private final CANSparkMax armFollowSparkMax;
  private final SparkMaxPIDController armPIDController;
  private final SparkMaxAbsoluteEncoder armEncoder;
  private final double armEncoderToArmGearRatio = 22.0 / 34.0;

  public ArmMotorIOSparkMax() {
    armSparkMax = new CANSparkMax(11, MotorType.kBrushless);
    armFollowSparkMax = new CANSparkMax(12, MotorType.kBrushless);

    if (SparkMaxBurnManager.shouldBurn()) {
      armSparkMax.restoreFactoryDefaults();
      armFollowSparkMax.restoreFactoryDefaults();
    }

    // Set followers
    armFollowSparkMax.follow(armSparkMax, true);

    // Set brake mode
    setBrakeMode(true);

    // Get and reset encoder objects
    armEncoder = armSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    armEncoder.setInverted(false); // TODO: Need to check this
    armEncoder.setZeroOffset(Units.radiansToRotations(0)); // TODO: Change this

    // Setup PID controllers
    armPIDController = armSparkMax.getPIDController();
    armPIDController.setFeedbackDevice(armEncoder);

    // Setup power parameters
    armSparkMax.enableVoltageCompensation(12.0);
    armFollowSparkMax.enableVoltageCompensation(12.0);

    // Setup CAN parameters
    armSparkMax.setCANTimeout(0);
    armFollowSparkMax.setCANTimeout(0);

    // Set scale conversion factors. Return native units and handle the unit conversion ourselves.
    armEncoder.setPositionConversionFactor(1);

    if (SparkMaxBurnManager.shouldBurn()) {
      armSparkMax.burnFlash();
      armFollowSparkMax.burnFlash();
    }
  }

  @Override
  public void updateInputs(ArmMotorIOInputs inputs) {
    // Arm state variables for logging
    inputs.armPositionRad =
        Units.rotationsToRadians(armEncoder.getPosition() * armEncoderToArmGearRatio);
    inputs.armVelocityRadS =
        Units.rotationsPerMinuteToRadiansPerSecond(
            armEncoder.getVelocity() * armEncoderToArmGearRatio);
    inputs.armAppliedVolts = armSparkMax.getAppliedOutput() * armSparkMax.getBusVoltage();
    inputs.armCurrentAmps =
        new double[] {armSparkMax.getOutputCurrent(), armFollowSparkMax.getOutputCurrent()};
    inputs.armTemp =
        new double[] {armSparkMax.getMotorTemperature(), armFollowSparkMax.getMotorTemperature()};

    if (inputs.isArmRunningPID) {
      armPIDController.setReference(
          inputs.armTargetPositionRad, ControlType.kPosition, 0, inputs.armFeedforward);
    }
  }

  @Override
  public void setVoltage(double volts) {
    armSparkMax.setVoltage(volts);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    armSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    armFollowSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setPIDGains(double kP, double kI, double kD) {
    armPIDController.setP(kP, 0);
    armPIDController.setI(kI, 0);
    armPIDController.setD(kD, 0);
  }
}
