package org.supurdueper.frc2023.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public interface Armavator {
  public static record ArmavatorPose(
      Rotation2d armAngle, double elevatorDistance, double armVelocity, double elevatorVelocity) {

    public TrapezoidProfile.State getArmProfileState() {
      return new TrapezoidProfile.State(armAngle.getRadians(), armVelocity);
    }

    public TrapezoidProfile.State getElevatorProfileState() {
      return new TrapezoidProfile.State(elevatorDistance, elevatorVelocity);
    }

    public static enum ArmavatorPreset {
      stowed(new ArmavatorPose(new Rotation2d(0), 0.0, 0.0, 0.0)),
      intake(new ArmavatorPose(new Rotation2d(Math.PI / 4), 0.5, 0.0, 0.0)),
      low(new ArmavatorPose(new Rotation2d(Math.PI / 4), 1.0, 0.0, 0.0)),
      midCone(new ArmavatorPose(new Rotation2d(Math.PI / 4), 0.5, 0.0, 0.0)),
      midCube(new ArmavatorPose(new Rotation2d(Math.PI / 4), 0.5, 0.0, 0.0)),
      highCone(new ArmavatorPose(new Rotation2d(Math.PI / 4), 0.5, 0.0, 0.0)),
      highCube(new ArmavatorPose(new Rotation2d(Math.PI / 4), 0.5, 0.0, 0.0)),
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
