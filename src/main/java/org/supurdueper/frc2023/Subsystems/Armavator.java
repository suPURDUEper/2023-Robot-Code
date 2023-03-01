package org.supurdueper.frc2023.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public interface Armavator {
  public static record ArmavatorPose(
      Rotation2d armAngle, double elevatorDistance, double armVelocity, double elevatorVelocity) {
    public static final double ELEVATOR_MIN = 0.05;
    public static final double ELEVATOR_SAFE = 0.17;
    public static final double ELEVATOR_INTAKE = 0.2;
    public static final double ELEVATOR_MAX = 0.6;
    public static final double ARM_SAFE_ANGLE = 0.2;
    public static final TrapezoidProfile.State ELEVATOR_SAFE_TARGET =
        new TrapezoidProfile.State(ArmavatorPose.ELEVATOR_SAFE, 0);

    public TrapezoidProfile.State getArmProfileState() {
      return new TrapezoidProfile.State(armAngle.getRadians(), armVelocity);
    }

    public TrapezoidProfile.State getElevatorProfileState() {
      return new TrapezoidProfile.State(elevatorDistance, elevatorVelocity);
    }

    public static enum ArmavatorPreset {
      stowed(new ArmavatorPose(new Rotation2d(-0.55), ELEVATOR_SAFE, 0.0, 0.0)),
      intakeCube(new ArmavatorPose(new Rotation2d(.185), .128, 0.0, 0.0)),
      intakeCone(new ArmavatorPose(new Rotation2d(-.39), .16, 0.0, 0.0)),
      low(new ArmavatorPose(new Rotation2d(.2), ELEVATOR_INTAKE, 0.0, 0.0)),
      midCone(new ArmavatorPose(new Rotation2d(2.1), ELEVATOR_MIN, 0.0, 0.0)),
      midCube(new ArmavatorPose(new Rotation2d(1.75), ELEVATOR_SAFE, 0.0, 0.0)),
      highCone(new ArmavatorPose(new Rotation2d(2), ELEVATOR_MAX, 0.0, 0.0)),
      highCube(new ArmavatorPose(new Rotation2d(1.57), ELEVATOR_MAX, 0.0, 0.0)),
      singleSubstationCone(new ArmavatorPose(new Rotation2d(1.9), 0.0, 0, 0));

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
