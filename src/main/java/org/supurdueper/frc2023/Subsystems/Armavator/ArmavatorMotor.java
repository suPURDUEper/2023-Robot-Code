package org.supurdueper.frc2023.subsystems.armavator;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class ArmavatorMotor {
  private final ArmavatorMotorIO io;
  private final ArmavatorMotorIOInputsAutoLogged inputs = new ArmavatorMotorIOInputsAutoLogged();

  private final double sprocketPitch = 1.75;
  private static final LoggedTunableNumber armKp = new LoggedTunableNumber("Arm/Motor/Kp");
  private static final LoggedTunableNumber armKd = new LoggedTunableNumber("Arm/Motor/Kd");
  private static final LoggedTunableNumber armKs = new LoggedTunableNumber("Arm/Motor/Ks");
  private static final LoggedTunableNumber armKg = new LoggedTunableNumber("Arm/Motor/Kg");
  private static final LoggedTunableNumber armKv = new LoggedTunableNumber("Arm/Motor/Kv");
  private static final LoggedTunableNumber elevatorKp =
      new LoggedTunableNumber("Elevator/Motor/Kp");
  private static final LoggedTunableNumber elevatorKd =
      new LoggedTunableNumber("Elevator/Motor/Kd");
  private static final LoggedTunableNumber elevatorKs =
      new LoggedTunableNumber("Elevator/Motor/Ks");
  private static final LoggedTunableNumber elevatorKv =
      new LoggedTunableNumber("Elevator/Motor/Kv");

  private ArmFeedforward armFeedforward = new ArmFeedforward(0.0, 0.0, 0.0);
  private SimpleMotorFeedforward elevatorFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
  private final PIDController armFeedback =
      new PIDController(0.0, 0.0, 0.0, Constants.loopPeriodSecs);
  private final PIDController elevatorFeedback =
      new PIDController(0, 0, 0, Constants.loopPeriodSecs);

  static {
    armKp.initDefault(0.1);
    armKd.initDefault(0.0);
    armKs.initDefault(0.12349);
    armKv.initDefault(0.13477);
    elevatorKp.initDefault(0.1);
    elevatorKd.initDefault(0.0);
    elevatorKs.initDefault(0.12349);
    elevatorKv.initDefault(0.13477);
  }

  public ArmavatorMotor(ArmavatorMotorIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateArmavatorInputs(inputs);
    Logger.getInstance().processInputs("Armavator/Motors", inputs);

    // Update controllers if tunable numbers have changed
    if (armKp.hasChanged(hashCode()) || armKd.hasChanged(hashCode())) {
      armFeedback.setPID(armKp.get(), 0.0, armKd.get());
    }
    if (elevatorKp.hasChanged(hashCode()) || elevatorKd.hasChanged(hashCode())) {
      elevatorFeedback.setPID(elevatorKp.get(), 0.0, elevatorKd.get());
    }
    if (armKs.hasChanged(hashCode()) || armKv.hasChanged(hashCode()) || armKg.hasChanged(hashCode())) {
      armFeedforward = new ArmFeedforward(armKs.get(),armKg.get(), armKv.get());
    }
    if (elevatorKs.hasChanged(hashCode()) || elevatorKv.hasChanged(hashCode())) {
      elevatorFeedforward = new SimpleMotorFeedforward(elevatorKs.get(), elevatorKv.get());
    }
  }

  public void Stop() {
    io.setArmVoltage(0);
    io.setElevatorVoltage(0);
  }

  public void setBrakeMode(boolean enabled) {
    io.setArmBrakeMode(enabled);
    io.setElevatorBrakeMode(enabled);
  }

  public double getArmVelocity() {
    return inputs.armVelocity;
  }

  public double getElevatorVelocity() {
    return inputs.elevatorVelocity * sprocketPitch;
  }
}
