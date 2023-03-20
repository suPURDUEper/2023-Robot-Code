package org.supurdueper.frc2023.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.frc2023.util.SparkMaxBurnManager;
import org.littletonrobotics.frc2023.util.SparkMaxPeriodicFrameConfig;

public class ElevatorMotorIOSparkMax implements ElevatorMotorIO {
  private final CANSparkMax elevatorSparkMax;
  private final CANSparkMax elevatorFollowSparkMax;
  private final SparkMaxPIDController elevatorPIDController;
  private final RelativeEncoder elevatorEncoder;

  private final double elevatorSprocketPitchDiameterIn = 1.751;
  private final double elevatorGearRatio = 12.0 / 1.0;

  public ElevatorMotorIOSparkMax() {
    elevatorSparkMax = new CANSparkMax(9, MotorType.kBrushless);
    elevatorFollowSparkMax = new CANSparkMax(10, MotorType.kBrushless);

    if (SparkMaxBurnManager.shouldBurn()) {
      elevatorSparkMax.restoreFactoryDefaults();
      elevatorFollowSparkMax.restoreFactoryDefaults();
    }

    elevatorEncoder = elevatorSparkMax.getEncoder();
    elevatorPIDController = elevatorSparkMax.getPIDController();

    for (int i = 0; i < SparkMaxBurnManager.configCount; i++) {
      // Configure status frames
      SparkMaxPeriodicFrameConfig.configLeaderFollower(elevatorSparkMax);
      SparkMaxPeriodicFrameConfig.configLeaderFollower(elevatorFollowSparkMax);

      // Set followers
      elevatorFollowSparkMax.follow(elevatorSparkMax, true);

      // Set brake mode
      setBrakeMode(true);

      // Get and reset encoder objects
      elevatorEncoder.setPosition(0);

      // Setup power parameters
      elevatorSparkMax.enableVoltageCompensation(12.0);
      elevatorFollowSparkMax.enableVoltageCompensation(12.0);
      elevatorSparkMax.setSmartCurrentLimit(40);
      elevatorFollowSparkMax.setSmartCurrentLimit(40);

      // Set soft limits
      elevatorSparkMax.setSoftLimit(SoftLimitDirection.kReverse, 0);
      elevatorSparkMax.setSoftLimit(
          SoftLimitDirection.kForward,
          (float) elevatorMetersToRotations(Elevator.elevatorMaxTravelM));
      elevatorSparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
      elevatorSparkMax.enableSoftLimit(
          SoftLimitDirection.kReverse, false); // Need to finish homing routine first

      // Set scale conversion factors. Return native units and handle the unit conversion ourselves.
      elevatorEncoder.setPositionConversionFactor(1);
    }

    // Setup CAN parameters
    elevatorSparkMax.setCANTimeout(SparkMaxBurnManager.configCANTimeout);
    elevatorFollowSparkMax.setCANTimeout(SparkMaxBurnManager.configCANTimeout);

    if (SparkMaxBurnManager.shouldBurn()) {
      elevatorSparkMax.burnFlash();
      elevatorFollowSparkMax.burnFlash();
    }
  }

  @Override
  public void updateInputs(ElevatorMotorIOInputs inputs) {
    // Elevator state variables for logging
    inputs.elevatorPositionM = elevatorRotationsToMeters(elevatorEncoder.getPosition());
    inputs.elevatorVelocityMS = elevatorRotationsToMeters(elevatorEncoder.getVelocity() / 60);
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

    if (inputs.isElevatorRunningPID) {
      double targetMotorRotations = elevatorMetersToRotations(inputs.elevatorTargetPositionM);
      elevatorPIDController.setReference(
          targetMotorRotations, ControlType.kPosition, 0, inputs.elevatorFeedforward);
    }
  }

  @Override
  public void setVoltage(double volts) {
    elevatorSparkMax.setVoltage(volts);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    elevatorSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    elevatorFollowSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setPIDGains(double kP, double kI, double kD) {
    elevatorPIDController.setP(kP, 0);
    elevatorPIDController.setI(kI, 0);
    elevatorPIDController.setD(kD, 0);
  }

  @Override
  public void resetEncoder() {
    elevatorEncoder.setPosition(0);
  }

  private double elevatorRotationsToMeters(double elevatorRotations) {
    return elevatorRotations
        / elevatorGearRatio
        * Math.PI
        * Units.inchesToMeters(elevatorSprocketPitchDiameterIn);
  }

  private double elevatorMetersToRotations(double elevatorMeters) {
    return Units.metersToInches(elevatorMeters)
        / (Math.PI * elevatorSprocketPitchDiameterIn)
        * elevatorGearRatio;
  }
}
