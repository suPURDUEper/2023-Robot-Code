package org.supurdueper.frc2023.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public interface Armavator {
  public static record ArmavatorPose(
      Rotation2d armAngle, double elevatorDistance, double armVelocity, double elevatorVelocity) {
    private static final double ELEVATOR_MIN = 0.2;
    private static final double ELEVATOR_NEUTRAL = 0.2;
    private static final double ELEVATOR_MAX = 0.6;

    public TrapezoidProfile.State getArmProfileState() {
      return new TrapezoidProfile.State(armAngle.getRadians(), armVelocity);
    }

    public TrapezoidProfile.State getElevatorProfileState() {
      return new TrapezoidProfile.State(elevatorDistance, elevatorVelocity);
    }

    public static enum ArmavatorPreset {
      stowed(new ArmavatorPose(new Rotation2d(-0.4), ELEVATOR_MIN, 0.0, 0.0)),
      intake(new ArmavatorPose(new Rotation2d(.2), ELEVATOR_NEUTRAL, 0.0, 0.0)),
      low(new ArmavatorPose(new Rotation2d(.2), ELEVATOR_NEUTRAL, 0.0, 0.0)),
      midCone(new ArmavatorPose(new Rotation2d(2.1), ELEVATOR_MIN, 0.0, 0.0)),
      midCube(new ArmavatorPose(new Rotation2d(1.75), ELEVATOR_NEUTRAL, 0.0, 0.0)),
      highCone(new ArmavatorPose(new Rotation2d(1.78), ELEVATOR_MAX, 0.0, 0.0)),
      highCube(new ArmavatorPose(new Rotation2d(1.57), ELEVATOR_MAX, 0.0, 0.0)),
      halfway(new ArmavatorPose(new Rotation2d(0), 0.3, 0, 0));

      private ArmavatorPose pose;

      private ArmavatorPreset(ArmavatorPose pose) {
        this.pose = pose;
      }

      public ArmavatorPose getPose() {
        return pose;
      }
    }
  }
}
