package org.supurdueper.frc2023.subsystems.elevator;

import org.littletonrobotics.frc2023.Constants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorMotorIOSim implements ElevatorMotorIO {

    private ElevatorSim elevatorSim = new ElevatorSim(DCMotor.getNEO(2), 3*4, Units.lbsToKilograms(25), Units.inchesToMeters(1.751), 0, 0.6, false);
    private double appliedVoltage = 0;
    private PIDController elevatorPIDController = new PIDController(0, 0, 0);

    @Override
    public void setPIDGains(double kP, double kI, double kD) {
        elevatorPIDController.setPID(kP, kI, kD);
    }

    @Override
    public void setVoltage(double volts) {
        appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
        elevatorSim.setInputVoltage(appliedVoltage);
        elevatorPIDController.reset();
    }

    @Override
    public void updateInputs(ElevatorMotorIOInputs inputs) {
        if (inputs.isElevatorRunningPID) {
            appliedVoltage = elevatorPIDController.calculate(inputs.elevatorPositionM, inputs.elevatorTargetPositionM);
            appliedVoltage += inputs.elevatorFeedforward;
            appliedVoltage = MathUtil.clamp(appliedVoltage, -12.0, 12.0);
        }
        inputs.elevatorAppliedVolts = appliedVoltage;
        elevatorSim.update(Constants.loopPeriodSecs);
        inputs.elevatorCurrentAmps = new double[] {elevatorSim.getCurrentDrawAmps(), elevatorSim.getCurrentDrawAmps()};
        inputs.elevatorPositionM = elevatorSim.getPositionMeters();
        inputs.elevatorVelocityMS = elevatorSim.getVelocityMetersPerSecond();
        inputs.elevatorTemp = new double[] {26.0, 26.0};
    }
}
