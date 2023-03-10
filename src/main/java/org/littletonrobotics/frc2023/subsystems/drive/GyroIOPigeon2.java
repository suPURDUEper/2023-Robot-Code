package org.littletonrobotics.frc2023.subsystems.drive;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.frc2023.Constants;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon;
  private final double[] yprDegrees = new double[3];
  private final double[] xyzDps = new double[3];

  public GyroIOPigeon2() {
    switch (Constants.getRobot()) {
      case ROBOT_2023C:
        pigeon = new Pigeon2(25);
        break;
      default:
        throw new RuntimeException("Invalid robot for GyroIOPigeon2");
    }

    pigeon.configFactoryDefault();
    pigeon.zeroGyroBiasNow();
    pigeon.configMountPose(-90, 0, 0);
    pigeon.setYaw(0.0);
  }

  public void updateInputs(GyroIOInputs inputs) {
    pigeon.getYawPitchRoll(yprDegrees);
    pigeon.getRawGyro(xyzDps);
    inputs.connected = pigeon.getLastError().equals(ErrorCode.OK);
    inputs.rollPositionRad = Units.degreesToRadians(yprDegrees[1]);
    inputs.pitchPositionRad = Units.degreesToRadians(-yprDegrees[2]);
    inputs.yawPositionRad = Units.degreesToRadians(yprDegrees[0]);
    inputs.rollVelocityRadPerSec = Units.degreesToRadians(xyzDps[1]);
    inputs.pitchVelocityRadPerSec = Units.degreesToRadians(-xyzDps[0]);
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(xyzDps[2]);
  }
}
