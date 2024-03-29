// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.util.SparkMaxBurnManager;
import org.littletonrobotics.frc2023.util.SparkMaxPeriodicFrameConfig;

public class ModuleIOSparkMax implements ModuleIO {
  private final CANSparkMax driveSparkMax;
  private final CANSparkMax turnSparkMax;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  private final AnalogInput turnAbsoluteEncoder;

  private final double driveAfterEncoderReduction = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
  private final double turnAfterEncoderReduction = 150.0 / 7.0;

  private final boolean isTurnMotorInverted = true;
  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOSparkMax(int index) {
    switch (Constants.getRobot()) {
      case ROBOT_2023C:
        switch (index) {
          case 0:
            driveSparkMax = new CANSparkMax(2, MotorType.kBrushless);
            turnSparkMax = new CANSparkMax(1, MotorType.kBrushless);
            turnAbsoluteEncoder = new AnalogInput(0);
            absoluteEncoderOffset = new Rotation2d(1.46);
            break;
          case 1:
            driveSparkMax = new CANSparkMax(4, MotorType.kBrushless);
            turnSparkMax = new CANSparkMax(3, MotorType.kBrushless);
            turnAbsoluteEncoder = new AnalogInput(1);
            absoluteEncoderOffset = new Rotation2d(-1.39);
            break;
          case 2:
            driveSparkMax = new CANSparkMax(8, MotorType.kBrushless);
            turnSparkMax = new CANSparkMax(7, MotorType.kBrushless);
            turnAbsoluteEncoder = new AnalogInput(2);
            absoluteEncoderOffset = new Rotation2d(0.35);
            break;
          case 3:
            driveSparkMax = new CANSparkMax(6, MotorType.kBrushless);
            turnSparkMax = new CANSparkMax(5, MotorType.kBrushless);
            turnAbsoluteEncoder = new AnalogInput(3);
            absoluteEncoderOffset = new Rotation2d(-0.36);
            break;

          default:
            throw new RuntimeException("Invalid module index for ModuleIOSparkMax");
        }
        break;
      default:
        throw new RuntimeException("Invalid robot for ModuleIOSparkMax");
    }

    if (SparkMaxBurnManager.shouldBurn()) {
      driveSparkMax.restoreFactoryDefaults();
      turnSparkMax.restoreFactoryDefaults();
    }

    driveSparkMax.setCANTimeout(SparkMaxBurnManager.configCANTimeout);
    turnSparkMax.setCANTimeout(SparkMaxBurnManager.configCANTimeout);

    driveEncoder = driveSparkMax.getEncoder();
    turnRelativeEncoder = turnSparkMax.getEncoder();

    for (int i = 0; i < SparkMaxBurnManager.configCount; i++) {
      SparkMaxPeriodicFrameConfig.configNotLeader(driveSparkMax);
      SparkMaxPeriodicFrameConfig.configNotLeader(turnSparkMax);

      turnSparkMax.setInverted(isTurnMotorInverted);

      driveSparkMax.setSmartCurrentLimit(30);
      turnSparkMax.setSmartCurrentLimit(30);
      driveSparkMax.enableVoltageCompensation(12.0);
      turnSparkMax.enableVoltageCompensation(12.0);

      driveEncoder.setPosition(0.0);
      driveEncoder.setMeasurementPeriod(10);
      driveEncoder.setAverageDepth(2);

      turnRelativeEncoder.setPosition(0.0);
    }

    driveSparkMax.setCANTimeout(0);
    turnSparkMax.setCANTimeout(0);

    if (SparkMaxBurnManager.shouldBurn()) {
      driveSparkMax.burnFlash();
      turnSparkMax.burnFlash();
    }
  }

  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad =
        Units.rotationsToRadians(driveEncoder.getPosition()) / driveAfterEncoderReduction;
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity())
            / driveAfterEncoderReduction;
    inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {driveSparkMax.getOutputCurrent()};
    inputs.driveTempCelcius = new double[] {driveSparkMax.getMotorTemperature()};

    inputs.turnAbsolutePositionRad =
        MathUtil.angleModulus(
            new Rotation2d(
                    turnAbsoluteEncoder.getVoltage()
                        / RobotController.getVoltage5V()
                        * 2.0
                        * Math.PI)
                .minus(absoluteEncoderOffset)
                .getRadians());
    inputs.turnPositionRad =
        Units.rotationsToRadians(turnRelativeEncoder.getPosition()) / turnAfterEncoderReduction;
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / turnAfterEncoderReduction;
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};
    inputs.turnTempCelcius = new double[] {turnSparkMax.getMotorTemperature()};
  }

  public void setDriveVoltage(double volts) {
    driveSparkMax.setVoltage(volts);
  }

  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  public void setDriveBrakeMode(boolean enable) {
    driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
