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

    armAbsoluteEncoder = armSparkMax.getAbsoluteEncoder(Type.kDutyCycle);

    for (int i = 0; i < SparkMaxBurnManager.configCount; i++) {
      // Set followers
      armFollowSparkMax.follow(armSparkMax, true);

      // Set CAN status frame frequency
      SparkMaxPeriodicFrameConfig.configLeaderFollower(armSparkMax);
      SparkMaxPeriodicFrameConfig.configLeaderFollower(armFollowSparkMax);

      // Speed up frame with absolute encoder position
      armSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 10);

      // Set brake mode
      setBrakeMode(true);

      // Get and reset encoder objects
      armAbsoluteEncoder.setInverted(false);
      armAbsoluteEncoder.setPositionConversionFactor(Units.rotationsToRadians(1));
      armAbsoluteEncoder.setZeroOffset(0.8394);
      // Sync with motor encoder so we can use the built-in soft limits on the motor controller
      // This should be equal to (2 * Pi) / armEncoderToMotorRatio, but I don't know why
      // that didn't work
      armSparkMax.getEncoder().setPositionConversionFactor(1 / 14.127);
      syncEncoders();

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
          SoftLimitDirection.kReverse, (float) -0.69); // Manually tuned due to backlash
      armSparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true);
    }

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
    double armPositionRad = wrapAbsoluteArmAngle(armAbsoluteEncoder.getPosition());
    inputs.armPositionRad = armPositionRad * armEncoderToArmGearRatio;
    // inputs.armPositionRad = armAbsoluteEncoder.getPosition();
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

  private double wrapAbsoluteArmAngle(double armAbsoluteEncoderPos) {
    if (armAbsoluteEncoderPos > Math.PI * 1.5) {
      armAbsoluteEncoderPos -= Math.PI * 2;
    }
    return armAbsoluteEncoderPos;
  }

  @Override
  public void syncEncoders() {
    armSparkMax.getEncoder().setPosition(wrapAbsoluteArmAngle(armAbsoluteEncoder.getPosition()));
  }
}
