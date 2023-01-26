package org.supurdueper.frc2023.subsystems.Armavator;

import org.littletonrobotics.junction.AutoLog;

public interface ArmavatorIO {
    @AutoLog
    public static class ArmavatorIOInputs {
        public double armPosition = 0.0;
        public double armVelocity = 0.0;
        public double armAppliedVolts = 0.0;
        public double[] arm1CurrentAmps = new double[] {};
        public double[] arm2CurrentAmps = new double[] {};
        public double[] arm1Temp = new double[] {};
        public double[] arm2Temp = new double[] {};

        public double elevatorPosition = 0.0;
        public double elevatorVelocity = 0.0;
        public double elevatorAppliedVolts = 0.0;
        public double[] elevatorCurrentAmps = new double[] {};
        public double[] elevatorTemp = new double[] {};
    }
    /* Updates the sets of loggable inputs */
    public default void updateArmavatorInputs(ArmavatorIOInputs Inputs) {}
    /* Run the arm motor at a specifc Voltage */
    public default void setArmVoltage(double volts) {}
}
