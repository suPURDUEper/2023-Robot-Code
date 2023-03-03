package org.supurdueper.frc2023.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.frc2023.util.SparkMaxBurnManager;
import org.littletonrobotics.frc2023.util.SparkMaxPeriodicFrameConfig;

public class ArmMotorIOSparkMax implements ArmMotorIO {
  private final CANSparkMax armSparkMax;
  private final CANSparkMax armFollowSparkMax;
  // private final SparkMaxPIDController armPIDController;
  private final SparkMaxAbsoluteEncoder armAbsoluteEncoder;
  private final double armEncoderToArmGearRatio = 22.0 / 34.0;
  private final double armEncoderToMotorRatio = (22.0 / 15.0) * 3.0 * 4.0 * 5.0;

  public ArmMotorIOSparkMax() {
    armSparkMax = new CANSparkMax(11, MotorType.kBrushless);
    armFollowSparkMax = new CANSparkMax(12, MotorType.kBrushless);

    if (SparkMaxBurnManager.shouldBurn()) {
      armSparkMax.restoreFactoryDefaults();
      armFollowSparkMax.restoreFactoryDefaults();
    }

    // Set followers
    armFollowSparkMax.follow(armSparkMax, true);

    // Set CAN status frame frequency
    SparkMaxPeriodicFrameConfig.configLeaderFollower(armSparkMax);
    SparkMaxPeriodicFrameConfig.configLeaderFollower(armFollowSparkMax);

    // Set brake mode
    setBrakeMode(true);

    // Get and reset encoder objects
    armAbsoluteEncoder = armSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    armAbsoluteEncoder.setInverted(true);
    armAbsoluteEncoder.setPositionConversionFactor(Units.rotationsToRadians(1));
    armAbsoluteEncoder.setZeroOffset(Units.rotationsToRadians(0.1893));
    armSparkMax.getEncoder().setPositionConversionFactor(1 / 14.127);
    double wrappedArmPos = armAbsoluteEncoder.getPosition();
    if (wrappedArmPos > Math.PI * 1.5) {
      wrappedArmPos -= Math.PI * 2;
    }
    armSparkMax.getEncoder().setPosition(wrappedArmPos);
    // Speed up status message that has encoder position
    armSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 10);

    // Setup PID controllers
    // armPIDController = armSparkMax.getPIDController();
    // armPIDController.setFeedbackDevice(armEncoder);

    // Setup power parameters
    armSparkMax.enableVoltageCompensation(12.0);
    armFollowSparkMax.enableVoltageCompensation(12.0);
    armSparkMax.setSmartCurrentLimit(30);
    armFollowSparkMax.setSmartCurrentLimit(30);

    armSparkMax.setSoftLimit(
        SoftLimitDirection.kForward, (float) (Arm.armMaxAngleRad / armEncoderToArmGearRatio));
    armSparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
    armSparkMax.setSoftLimit(
        SoftLimitDirection.kReverse, (float) -0.5); // Manually tuned due to backlash
    armSparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true);

    // Setup CAN parameters
    armSparkMax.setCANTimeout(SparkMaxBurnManager.configCANTimeout);
    armFollowSparkMax.setCANTimeout(SparkMaxBurnManager.configCANTimeout);

    if (SparkMaxBurnManager.shouldBurn()) {
      armSparkMax.burnFlash();
      armFollowSparkMax.burnFlash();
    }
  }

  @Override
  public void updateInputs(ArmMotorIOInputs inputs) {
    // Arm state variables for logging
    double armPositionRad = armAbsoluteEncoder.getPosition();
    if (armPositionRad > Math.PI * 1.5) {
      armPositionRad -= Math.PI * 2;
    }
    inputs.armPositionRad = armPositionRad * armEncoderToArmGearRatio;
    inputs.armVelocityRadS =
        Units.rotationsPerMinuteToRadiansPerSecond(
            armAbsoluteEncoder.getVelocity() * armEncoderToArmGearRatio);
    inputs.armAppliedVolts = armSparkMax.getAppliedOutput() * armSparkMax.getBusVoltage();
    inputs.armCurrentAmps =
        new double[] {armSparkMax.getOutputCurrent(), armFollowSparkMax.getOutputCurrent()};
    inputs.armTemp =
        new double[] {armSparkMax.getMotorTemperature(), armFollowSparkMax.getMotorTemperature()};

    SmartDashboard.putNumber("Arm Raw Encoder value", armSparkMax.getEncoder().getPosition());

    // if (inputs.isArmRunningPID) {
    //   armPIDController.setReference(
    //       inputs.armTargetPositionRad, CANSparkMax.ControlType.kPosition, 0,
    // inputs.armFeedforward);
    // }
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
    // armPIDController.setP(kP, 0);
    // armPIDController.setI(kI, 0);
    // armPIDController.setD(kD, 0);
  }
}
