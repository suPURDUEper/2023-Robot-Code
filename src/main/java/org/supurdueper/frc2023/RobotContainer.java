// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.frc2023;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Supplier;
import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.Constants.Mode;
import org.littletonrobotics.frc2023.Robot;
import org.littletonrobotics.frc2023.commands.DriveWithJoysticks;
import org.littletonrobotics.frc2023.subsystems.drive.Drive;
import org.littletonrobotics.frc2023.subsystems.drive.GyroIO;
import org.littletonrobotics.frc2023.subsystems.drive.GyroIOPigeon2;
import org.littletonrobotics.frc2023.subsystems.drive.ModuleIO;
import org.littletonrobotics.frc2023.subsystems.drive.ModuleIOSim;
import org.littletonrobotics.frc2023.subsystems.drive.ModuleIOSparkMax;
import org.littletonrobotics.frc2023.util.Alert;
import org.littletonrobotics.frc2023.util.Alert.AlertType;
import org.littletonrobotics.frc2023.util.SparkMaxBurnManager;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.supurdueper.frc2023.commands.IntakeCone;
import org.supurdueper.frc2023.commands.IntakeCube;
import org.supurdueper.frc2023.commands.Score;
import org.supurdueper.frc2023.commands.ConeAndBalanceAuto;
import org.supurdueper.frc2023.commands.arm.MoveArmWithJoystick;
import org.supurdueper.frc2023.commands.arm.SyncArmEncoders;
import org.supurdueper.frc2023.commands.armavator.ArmavatorGoToPose;
import org.supurdueper.frc2023.commands.drive.DriveWithLockedRotation;
import org.supurdueper.frc2023.commands.elevator.MoveElevatorWithJoystick;
import org.supurdueper.frc2023.commands.elevator.ResetElevatorPosition;
import org.supurdueper.frc2023.subsystems.Armavator.ArmavatorPose.ArmavatorPreset;
import org.supurdueper.frc2023.subsystems.arm.Arm;
import org.supurdueper.frc2023.subsystems.arm.ArmMotorIOSparkMax;
import org.supurdueper.frc2023.subsystems.elevator.Elevator;
import org.supurdueper.frc2023.subsystems.elevator.ElevatorMotorIOSparkMax;
import org.supurdueper.frc2023.subsystems.intake.Intake;
import org.supurdueper.frc2023.subsystems.intake.IntakeIOTalonFX;

public class RobotContainer {

  // Subsystems
  private Drive drive;
  private Elevator elevator;
  private Arm arm;
  private Intake intake;

  // OI objects
  private CommandXboxController driver = new CommandXboxController(0);
  private CommandXboxController operator = new CommandXboxController(1);
  private Alert driverDisconnected =
      new Alert("Driver controller is not connected (port 0).", AlertType.WARNING);
  private Alert operatorDisconnected =
      new Alert("Operator controller is not connected (port 1).", AlertType.WARNING);

  // Choosers
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  public RobotContainer() {
    // Check if flash should be burned
    SparkMaxBurnManager.update();

    // Instantiate active subsystems
    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case ROBOT_2023C:
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOSparkMax(0),
                  new ModuleIOSparkMax(1),
                  new ModuleIOSparkMax(2),
                  new ModuleIOSparkMax(3));
          elevator = new Elevator(new ElevatorMotorIOSparkMax());
          arm = new Arm(new ArmMotorIOSparkMax());
          intake = new Intake(new IntakeIOTalonFX());
          break;
        case ROBOT_SIMBOT:
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim());
          break;
      }
    }

    // Instantiate missing subsystems
    drive =
        drive != null
            ? drive
            : new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

    // Set up subsystems

    // Set up auto routines
    autoChooser.addDefaultOption("Do Nothing", null);
    autoChooser.addDefaultOption(
        "Reset Odometry", new InstantCommand(() -> drive.setPose(new Pose2d())));

    // Alert if in tuning mode
    if (Constants.tuningMode) {
      new Alert("Tuning mode active, do not use in competition.", AlertType.INFO).set(true);
    }

    // Instantiate OI classes and bind buttons
    bindControls();
  }

  /** Updates the alerts for disconnected controllers. */
  public void checkControllers() {
    driverDisconnected.set(
        !DriverStation.isJoystickConnected(driver.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(driver.getHID().getPort()));
    operatorDisconnected.set(
        !DriverStation.isJoystickConnected(operator.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(operator.getHID().getPort()));
  }

  /**
   * This method scans for any changes to the connected joystick. If anything changed, it creates
   * new OI objects and binds all of the buttons to commands.
   */
  public void bindControls() {
    // Rely on our custom alerts for disconnected controllers
    DriverStation.silenceJoystickConnectionWarning(true);

    // Driver
    Trigger rotateTo0 = driver.y();
    Trigger rotateTo90 = driver.x();
    Trigger rotateTo180 = driver.a();
    Trigger rotateTo270 = driver.b();
    Supplier<Double> driveTranslationX = invertJoystick(driver::getLeftX);
    Supplier<Double> driveTranslationY = invertJoystick(driver::getLeftY);
    Supplier<Double> driveRotate = invertJoystick(driver::getRightX);
    Trigger score = driver.leftBumper();
    Trigger driveAutoAim = driver.rightBumper();
    Trigger swerveXMode = driver.povDown();
    Trigger slowMode = driver.leftTrigger(0.2).or(driver.rightTrigger(0.2));

    // Operator
    Supplier<Double> manualArmControl = invertJoystick(operator::getLeftY);
    Supplier<Double> manualElevatorControl = invertJoystick(operator::getRightY);
    Trigger armavatorLow = operator.a();
    Trigger armavatorMid = operator.b();
    Trigger armavatorHigh = operator.y();
    Trigger armavatorStow = operator.x();
    Trigger manualIntakeCube = operator.leftTrigger(0.2);
    Trigger manualIntakeCone = operator.rightTrigger(0.2);
    Trigger intakeCube = operator.leftBumper();
    Trigger intakeCone = operator.rightBumper();
    Trigger intakeOff = operator.back();
    Trigger singleStationConeIntake = operator.povLeft();
    Trigger doubleStationConeIntake = operator.povUp();

    // *** DRIVER CONTROLS ***
    drive.setDefaultCommand(
        new DriveWithJoysticks(
            drive,
            driveTranslationY,
            driveTranslationX,
            driveRotate,
            slowMode::getAsBoolean, // Slow mode
            () -> false, // Switch to robot relative driving
            () -> 0.0)); // Limit acceleration based on arm extension percentage

    rotateTo0.whileTrue(
        new DriveWithLockedRotation(
            drive, driveTranslationY, driveTranslationX, Units.degreesToRadians(0), slowMode));
    rotateTo90.whileTrue(
        new DriveWithLockedRotation(
            drive, driveTranslationY, driveTranslationX, Units.degreesToRadians(90), slowMode));
    rotateTo180.whileTrue(
        new DriveWithLockedRotation(
            drive, driveTranslationY, driveTranslationX, Units.degreesToRadians(180), slowMode));
    rotateTo270.whileTrue(
        new DriveWithLockedRotation(
            drive, driveTranslationY, driveTranslationX, Units.degreesToRadians(-90), slowMode));

    swerveXMode.whileTrue(
        new StartEndCommand(() -> drive.setXMode(true), () -> drive.setXMode(false), drive));

    score.onTrue(
        new Score(intake)
            .withTimeout(0.5)
            .andThen(
                Commands.either(
                        new ArmavatorGoToPose(ArmavatorPreset.midCube.getPose(), arm, elevator),
                        new ArmavatorGoToPose(ArmavatorPreset.midCone.getPose(), arm, elevator),
                        intake::hasCube)
                    .unless(() -> arm.getArmPosition().getRadians() < 1.0)));

    // driveAutoAim.whileTrue(new DriveSnapToPose(drive, new Pose2d(), driveTranslationX,
    // driveTranslationY));

    // *** OPERATOR CONTROLS ***
    armavatorHigh.onTrue(
        Commands.either(
            new ArmavatorGoToPose(ArmavatorPreset.highCube.getPose(), arm, elevator),
            new ArmavatorGoToPose(ArmavatorPreset.highCone.getPose(), arm, elevator),
            intake::hasCube));

    armavatorMid.onTrue(
        Commands.either(
            new ArmavatorGoToPose(ArmavatorPreset.midCube.getPose(), arm, elevator),
            new ArmavatorGoToPose(ArmavatorPreset.midCone.getPose(), arm, elevator),
            intake::hasCube));

    armavatorStow.onTrue(new ArmavatorGoToPose(ArmavatorPreset.stowed.getPose(), arm, elevator));

    intakeCube.onTrue(
        new ArmavatorGoToPose(ArmavatorPreset.intakeCube.getPose(), arm, elevator)
            .andThen(new IntakeCube(intake))
            .andThen(new ArmavatorGoToPose(ArmavatorPreset.stowed.getPose(), arm, elevator)));

    manualIntakeCone.whileTrue(
        new StartEndCommand(
            () -> intake.setIntakeMode(Intake.Mode.INTAKE_CONE),
            () -> intake.setIntakeMode(Intake.Mode.NOT_RUNNING),
            intake));

    manualIntakeCube.whileTrue(
        new StartEndCommand(
            () -> intake.setIntakeMode(Intake.Mode.INTAKE_CUBE),
            () -> intake.setIntakeMode(Intake.Mode.NOT_RUNNING),
            intake));

    intakeCone.onTrue(
        new ArmavatorGoToPose(ArmavatorPreset.intakeCone.getPose(), arm, elevator)
            .andThen(new IntakeCone(intake))
            .andThen(new ArmavatorGoToPose(ArmavatorPreset.stowed.getPose(), arm, elevator)));

    intakeOff.onTrue(new InstantCommand(() -> intake.setIntakeMode(Intake.Mode.NOT_RUNNING)));

    operator.start().onTrue(new ResetElevatorPosition(elevator));

    singleStationConeIntake.onTrue(
        new ArmavatorGoToPose(ArmavatorPreset.singleSubstationCone.getPose(), arm, elevator)
            .andThen(new IntakeCone(intake)));

    // Change this later - touching joystick should interrupt command
    elevator.setDefaultCommand(new MoveElevatorWithJoystick(elevator, manualElevatorControl));
    arm.setDefaultCommand(new MoveArmWithJoystick(arm, manualArmControl));
  }

  /** Passes the autonomous command to the {@link Robot} class. */
  public Command getAutonomousCommand() {
    return new ConeAndBalanceAuto(drive, intake, arm, elevator);
  }

  public Supplier<Double> invertJoystick(Supplier<Double> joystick) {
    return () -> joystick.get() * -1;
  }

  // Method to get this command so we can use it in Robot.java
  public Command getSyncArmEncoderCommand() {
    return new SyncArmEncoders(arm);
  }
}
