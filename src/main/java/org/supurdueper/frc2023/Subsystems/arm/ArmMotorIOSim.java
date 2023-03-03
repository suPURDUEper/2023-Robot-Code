package org.supurdueper.frc2023.subsystems.arm;

import org.littletonrobotics.frc2023.Constants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmMotorIOSim implements ArmMotorIO {

    private SingleJointedArmSim armSim = new SingleJointedArmSim(DCMotor.getNEO(2), (34 / 15)*3*4*5, 0.5, 0.5, Arm.armMinAngleRad, Arm.armMaxAngleRad, true);
    private double armAppliedVolts = 0.0;

    @Override
    public void setVoltage(double voltage) {
        armAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
        armSim.setInputVoltage(armAppliedVolts);
    }

    @Override
    public void updateInputs(ArmMotorIOInputs inputs) {
        armSim.update(Constants.loopPeriodSecs);
        inputs.armAppliedVolts = armAppliedVolts;
        inputs.armCurrentAmps = new double[] {armSim.getCurrentDrawAmps(), armSim.getCurrentDrawAmps()};
        inputs.armPositionRad = armSim.getAngleRads();
        inputs.armVelocityRadS = armSim.getVelocityRadPerSec();
        inputs.armTemp = new double [] {26.0, 26.0};
    }

    
    
}
