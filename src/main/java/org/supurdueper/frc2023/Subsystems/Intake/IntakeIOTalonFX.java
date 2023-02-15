package org.supurdueper.frc2023.Subsystems.Intake;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWM;
import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.util.SparkMaxBurnManager;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX     Roller;
  private final SupplyCurrentLimitConfiguration IntakeCurrentConfig;

  private final PWM armAbsoluteEncoder;
  private final Encoder armRelativeEncoder;
  private final RelativeEncoder armInternalEncoder;

  private final boolean armInvert;
  private final boolean armExternalEncoderInvert;
  private final boolean rollerInvert;
  private final double armInternalEncoderReduction;
  private final Rotation2d armAbsoluteEncoderOffset;

  public IntakeIOTalonFX() {
    switch (Constants.getRobot()) {
      case ROBOT_2023C:
        Roller = new TalonFX(0);
        IntakeCurrentConfig = new SupplyCurrentLimitConfiguration(true,15, 15, 0);

        rollerInvert = false;


      default:
        throw new RuntimeException("Invalid robot for CubeIntakeIOSparkMax!");
    }

  /*   if (SparkMaxBurnManager.shouldBurn()) {
      armSparkMax.restoreFactoryDefaults();
      armSparkmax2.restoreFactoryDefaults();
    } */



    Roller.configSupplyCurrentLimit(IntakeCurrentConfig);

    Roller.configVoltageCompSaturation(10.0);
    Roller.enableVoltageCompensation(true);

  /*  if (SparkMaxBurnManager.shouldBurn()) {
      armSparkMax.burnFlash();
      armSparkmax2.burnFlash();
    } */
  }

  @Override
  public void updateInputs(CubeIntakeIOInputs inputs) {
  

    inputs.rollerAppliedVolts = rollerTalonFX.getAppliedOutput() * rollerTalonFX.getBusVoltage();
    //inputs.rollerCurrentAmps = new double[] {rollerSparkMax.getOutputCurrent()};
  }

 /*  @Override
  public void setArmVoltage(double voltage) {
    armSparkMax.setVoltage(voltage);
  }

  @Override
  public void setRollerVoltage(double voltage) {
    rollerSparkMax.setVoltage(voltage);
  } */
}