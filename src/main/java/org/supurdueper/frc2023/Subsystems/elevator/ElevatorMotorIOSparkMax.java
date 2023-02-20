package org.supurdueper.frc2023.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.frc2023.util.SparkMaxBurnManager;

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

    // Set followers
    elevatorFollowSparkMax.follow(elevatorSparkMax, true);

    // Set brake mode
    setBrakeMode(true);

    // Get and reset encoder objects
    elevatorEncoder = elevatorSparkMax.getEncoder();
    elevatorEncoder.setPosition(0);

    // Setup PID controllers
    elevatorPIDController = elevatorSparkMax.getPIDController();

    // Setup power parameters
    elevatorSparkMax.setSmartCurrentLimit(40);
    elevatorFollowSparkMax.setSmartCurrentLimit(40);

    // Setup CAN parameters
    elevatorSparkMax.setCANTimeout(0);
    elevatorFollowSparkMax.setCANTimeout(0);

    // Set scale conversion factors. Return native units and handle the unit conversion ourselves.
    elevatorEncoder.setPositionConversionFactor(1);

    if (SparkMaxBurnManager.shouldBurn()) {
      elevatorSparkMax.burnFlash();
      elevatorFollowSparkMax.burnFlash();
    }
  }

  @Override
  public void updateInputs(ElevatorMotorIOInputs inputs) {
    // Elevator state variables for logging
    inputs.elevatorPositionM =
        elevatorEncoder.getPosition()
            / elevatorGearRatio
            * Math.PI
            * Units.inchesToMeters(elevatorSprocketPitchDiameterIn);
    inputs.elevatorVelocityMS =
        (elevatorEncoder.getVelocity() / 60)
            / elevatorGearRatio
            * Math.PI
            * Units.inchesToMeters(elevatorSprocketPitchDiameterIn);
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

      double targetMotorRotations =
          Units.metersToInches(inputs.elevatorTargetPositionM)
              / (Math.PI * elevatorSprocketPitchDiameterIn)
              * elevatorGearRatio;
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
}
