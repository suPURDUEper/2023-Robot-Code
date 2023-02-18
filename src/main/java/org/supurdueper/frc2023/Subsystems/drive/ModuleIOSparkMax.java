package org.supurdueper.frc2023.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.util.SparkMaxBurnManager;

public class ModuleIOSparkMax implements ModuleIO {
  private final CANSparkMax driveSparkMax;
  private final CANSparkMax turnSparkMax;

<<<<<<< HEAD
  //  private final SparkMaxDerivedVelocityController driveDerivedVelocityController;
=======
  private final RelativeEncoder driveEncoder;
>>>>>>> main
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
            absoluteEncoderOffset = new Rotation2d(-0.138844);
            break;
          case 1:
            driveSparkMax = new CANSparkMax(4, MotorType.kBrushless);
            turnSparkMax = new CANSparkMax(3, MotorType.kBrushless);
            turnAbsoluteEncoder = new AnalogInput(1);
            absoluteEncoderOffset = new Rotation2d(-0.075444);
            break;
          case 2:
            driveSparkMax = new CANSparkMax(8, MotorType.kBrushless);
            turnSparkMax = new CANSparkMax(7, MotorType.kBrushless);
            turnAbsoluteEncoder = new AnalogInput(3);
            absoluteEncoderOffset = new Rotation2d(2.659570);
            break;
          case 3:
            driveSparkMax = new CANSparkMax(6, MotorType.kBrushless);
            turnSparkMax = new CANSparkMax(5, MotorType.kBrushless);
            turnAbsoluteEncoder = new AnalogInput(2);
            absoluteEncoderOffset = new Rotation2d(2.417932);
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

    turnSparkMax.setInverted(isTurnMotorInverted);

    driveSparkMax.setSmartCurrentLimit(30);
    turnSparkMax.setSmartCurrentLimit(30);
    driveSparkMax.enableVoltageCompensation(12.0);
    turnSparkMax.enableVoltageCompensation(12.0);

    driveSparkMax.getEncoder().setPosition(0.0);
<<<<<<< HEAD
    //    driveDerivedVelocityController = new SparkMaxDerivedVelocityController(driveSparkMax);
=======
    driveEncoder = driveSparkMax.getEncoder();
    driveEncoder.setMeasurementPeriod(10);
    driveEncoder.setAverageDepth(2);

>>>>>>> main
    turnRelativeEncoder = turnSparkMax.getEncoder();
    turnRelativeEncoder.setPosition(0.0);

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
        new Rotation2d(
                turnAbsoluteEncoder.getVoltage() / RobotController.getVoltage5V() * 2.0 * Math.PI)
            .minus(absoluteEncoderOffset)
            .getRadians();
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
