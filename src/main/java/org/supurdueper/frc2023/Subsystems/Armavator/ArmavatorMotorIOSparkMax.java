package org.supurdueper.frc2023.subsystems.armavator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
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

  private final SparkMaxAbsoluteEncoder armEncoder;
  private final RelativeEncoder elevatorEncoder;

  private final double sprocketPitchDiameterIn = 1.751;
  private final double armEncoderToArmGearRatio = 22.0 / 34.0;

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

    // Set followers
    elevatorFollowSparkMax.follow(elevatorSparkMax, true);
    armFollowSparkMax.follow(armSparkMax, true);

    // Set brake mode
    setArmBrakeMode(true);
    setElevatorBrakeMode(true);

    // Get and reset encoder objects
    armEncoder = armSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    armEncoder.setInverted(false); // TODO: Need to check this
    armEncoder.setZeroOffset(Units.radiansToRotations(0)); // TODO: Change this
    elevatorEncoder = elevatorSparkMax.getEncoder();
    elevatorEncoder.setInverted(false); // TODO: Need to check this
    elevatorEncoder.setPosition(0);

    // Setup PID controllers
    elevatorPIDController = elevatorSparkMax.getPIDController();
    armPIDController = armSparkMax.getPIDController();
    armPIDController.setFeedbackDevice(armEncoder);

    // Setup power parameters
    elevatorSparkMax.setSmartCurrentLimit(40);
    elevatorFollowSparkMax.setSmartCurrentLimit(40);
    armSparkMax.enableVoltageCompensation(12.0);
    armFollowSparkMax.enableVoltageCompensation(12.0);

    // Setup CAN parameters
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

  @Override
  public void updateArmavatorInputs(ArmavatorMotorIOInputs inputs) {
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

    // Elevator state variables for logging
    inputs.elevatorPositionM =
        Units.rotationsToRadians(elevatorEncoder.getPosition())
            * Units.inchesToMeters(sprocketPitchDiameterIn);
    inputs.elevatorVelocityMS =
        Units.rotationsPerMinuteToRadiansPerSecond(elevatorEncoder.getVelocity())
            * Units.inchesToMeters(sprocketPitchDiameterIn);
    inputs.elevatorAppliedVolts =
        elevatorSparkMax.getAppliedOutput() * elevatorSparkMax.getBusVoltage();
    inputs.elevatorCurrentAmps =
        new double[] {
          elevatorSparkMax.getOutputCurrent(), elevatorFollowSparkMax.getOutputCurrent()
        };
    inputs.elevatorTemp =
        new double[] {
          elevatorSparkMax.getMotorTemperature(), elevatorFollowSparkMax.getMotorTemperature()
        };

    // PID is always running, update the goal position every loop
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
    armFollowSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setElevatorBrakeMode(boolean enable) {
    elevatorSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    elevatorFollowSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
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
