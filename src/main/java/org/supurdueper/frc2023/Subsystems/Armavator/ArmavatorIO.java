package org.supurdueper.frc2023.subsystems.Armavator;

import org.littletonrobotics.junction.AutoLog;

public interface ArmavatorIO {
    @AutoLog
    public static class ArmavatorIOInputs {
        public double armPosition = 0.0;
    }
    
}
