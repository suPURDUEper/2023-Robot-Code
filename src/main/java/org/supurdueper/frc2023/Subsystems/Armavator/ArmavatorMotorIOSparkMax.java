package org.supurdueper.frc2023.subsystems.armavator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.frc2023.util.SparkMaxBurnManager;

public class ArmavatorMotorIOSparkMax implements ArmavatorMotorIO {
  private final CANSparkMax armSparkMax;
  private final CANSparkMax armFollowSparkMax;
  private final CANSparkMax elevatorSparkMax;
  private final CANSparkMax elevatorFollowSparkMax;

  private final SparkMaxPIDController armPIDController;
  private final SparkMaxPIDController elevatorPIDController;

  private final RelativeEncoder armEncoder;
  private final RelativeEncoder elevatorEncoder;

  private final double sprocketPitchDiameterIn = 1.751;

  public ArmavatorMotorIOSparkMax() {
    elevatorSparkMax = new CANSparkMax(9, MotorType.kBrushless);
    elevatorFollowSparkMax = new CANSparkMax(10, MotorType.kBrushless);
    armSparkMax = new CANSparkMax(11, MotorType.kBrushless);
    armFollowSparkMax = new CANSparkMax(12, MotorType.kBrushless);

    if (SparkMaxBurnManager.shouldBurn()) {
      elevatorSparkMax.restoreFactoryDefaults();
      elevatorFollowSparkMax.restoreFactoryDefaults();
      armSparkMax.restoreFactoryDefaults();
      armFollowSparkMax.restoreFactoryDefaults();
    }

    elevatorFollowSparkMax.follow(elevatorSparkMax);
    armFollowSparkMax.follow(armSparkMax);

    elevatorPIDController = elevatorSparkMax.getPIDController();
    armPIDController = armSparkMax.getPIDController();

    armEncoder = armSparkMax.getEncoder();
    armEncoder.setPosition(0);
    elevatorEncoder = elevatorSparkMax.getEncoder();
    elevatorEncoder.setPosition(0);

    elevatorSparkMax.setSmartCurrentLimit(40);
    elevatorFollowSparkMax.setSmartCurrentLimit(40);
    armSparkMax.enableVoltageCompensation(12.0);
    armFollowSparkMax.enableVoltageCompensation(12.0);

    elevatorSparkMax.setCANTimeout(0);
    elevatorFollowSparkMax.setCANTimeout(0);
    armSparkMax.setCANTimeout(0);
    armFollowSparkMax.setCANTimeout(0);

    if (SparkMaxBurnManager.shouldBurn()) {
      elevatorSparkMax.burnFlash();
      elevatorFollowSparkMax.burnFlash();
      armSparkMax.burnFlash();
      armFollowSparkMax.burnFlash();
    }
  }

  public void updateInputs(ArmavatorMotorIOInputs inputs) {
    inputs.armPositionRad = Units.rotationsToRadians(armSparkMax.getEncoder().getPosition());
    inputs.elevatorPositionM =
        Units.rotationsToRadians(elevatorSparkMax.getEncoder().getPosition())
            * Units.inchesToMeters(sprocketPitchDiameterIn);

    inputs.armVelocityRadS = armSparkMax.getEncoder().getVelocity();
    inputs.elevatorVelocityMS =
        Units.rotationsPerMinuteToRadiansPerSecond(elevatorSparkMax.getEncoder().getVelocity())
            * Units.inchesToMeters(sprocketPitchDiameterIn);

    inputs.armAppliedVolts = armSparkMax.getAppliedOutput() * armSparkMax.getBusVoltage();
    inputs.elevatorAppliedVolts =
        elevatorSparkMax.getAppliedOutput() * elevatorSparkMax.getBusVoltage();

    inputs.armCurrentAmps =
        new double[] {armSparkMax.getOutputCurrent(), armFollowSparkMax.getOutputCurrent()};
    inputs.elevatorCurrentAmps =
        new double[] {
          elevatorSparkMax.getOutputCurrent(), elevatorFollowSparkMax.getOutputCurrent()
        };

    inputs.armTemp =
        new double[] {armSparkMax.getMotorTemperature(), armFollowSparkMax.getMotorTemperature()};
    inputs.elevatorTemp =
        new double[] {
          elevatorSparkMax.getMotorTemperature(), elevatorFollowSparkMax.getMotorTemperature()
        };

    elevatorPIDController.setReference(
        inputs.elevatorTargetPositionM, ControlType.kPosition, 0, inputs.elevatorFeedforward);
    armPIDController.setReference(
        inputs.armTargetPositionRad, ControlType.kPosition, 0, inputs.armFeedforward);
  }

  @Override
  public void setArmVoltage(double volts) {
    armSparkMax.setVoltage(volts);
  }

  @Override
  public void setElevatorVoltage(double volts) {
    elevatorSparkMax.setVoltage(volts);
  }

  @Override
  public void setArmBrakeMode(boolean enable) {
    armSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setArmPIDGains(double kP, double kI, double kD) {
    armPIDController.setP(kP, 0);
    armPIDController.setI(kI, 0);
    armPIDController.setD(kD, 0);
  }

  @Override
  public void setElevatorPIDGains(double kP, double kI, double kD) {
    elevatorPIDController.setP(kP, 0);
    elevatorPIDController.setI(kI, 0);
    elevatorPIDController.setD(kD, 0);
  }
}
