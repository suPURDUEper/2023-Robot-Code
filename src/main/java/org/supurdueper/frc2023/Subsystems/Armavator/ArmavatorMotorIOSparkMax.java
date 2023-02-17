package org.supurdueper.frc2023.subsystems.armavator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.RobotController;

public class ArmavatorMotorIOSparkMax implements ArmavatorMotorIO {
  private final CANSparkMax armSparkMax;
  private final CANSparkMax armFollowSparkMax;
  private final CANSparkMax elevatorSparkMax;
  private final CANSparkMax elevatorFollowSparkMax;

  private final SparkMaxPIDController armPIDController;
  private final SparkMaxPIDController elevatorPIDController;

  private final RelativeEncoder armEncoder;
  private final RelativeEncoder elevatorEncoder;

  private final double sprocketPitch = 1.75;

  public ArmavatorMotorIOSparkMax() {
    elevatorSparkMax = new CANSparkMax(9, MotorType.kBrushless);
    elevatorFollowSparkMax = new CANSparkMax(10, MotorType.kBrushless);
    armSparkMax = new CANSparkMax(11, MotorType.kBrushless);
    armFollowSparkMax = new CANSparkMax(12, MotorType.kBrushless);

    elevatorFollowSparkMax.follow(elevatorSparkMax);
    armFollowSparkMax.follow(armSparkMax);

    elevatorPIDController = elevatorSparkMax.getPIDController();
    armPIDController = armSparkMax.getPIDController();

    armEncoder = armSparkMax.getEncoder();
    armEncoder.setPosition(0);
    elevatorEncoder = elevatorSparkMax.getEncoder();
    elevatorEncoder.setPosition(0);
  }

  public void updateInputs(ArmavatorMotorIOInputs inputs) {
    inputs.armPosition = armSparkMax.getEncoder().getPosition();
    inputs.elevatorPosition = elevatorSparkMax.getEncoder().getPosition();

    inputs.armVelocity = armSparkMax.getEncoder().getVelocity();
    inputs.elevatorVelocity = elevatorSparkMax.getEncoder().getVelocity() * sprocketPitch;

    inputs.armAppliedVolts = armSparkMax.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.elevatorAppliedVolts =
        elevatorSparkMax.getAppliedOutput() * RobotController.getBatteryVoltage();

    inputs.armCurrentAmps = new double[] {armSparkMax.getOutputCurrent()};
    inputs.elevatorCurrentAmps = new double[] {elevatorSparkMax.getOutputCurrent()};

    inputs.armTemp = new double[] {armSparkMax.getMotorTemperature()};
    inputs.elevatorTemp = new double[] {elevatorSparkMax.getMotorTemperature()};

    elevatorPIDController.setReference(
        inputs.elevatorTargetPosition, ControlType.kPosition, 0, inputs.elevatorFeedforward);
    armPIDController.setReference(
        inputs.armTargetPosition, ControlType.kPosition, 0, inputs.armFeedforward);
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