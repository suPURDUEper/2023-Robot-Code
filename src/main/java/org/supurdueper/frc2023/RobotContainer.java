// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.frc2023;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.frc2023.Constants;
import org.littletonrobotics.frc2023.Constants.Mode;
import org.littletonrobotics.frc2023.util.Alert;
import org.littletonrobotics.frc2023.util.Alert.AlertType;
import org.littletonrobotics.frc2023.util.SparkMaxBurnManager;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.supurdueper.frc2023.commands.DriveWithJoysticks;
import org.supurdueper.frc2023.commands.arm.ArmGoToPose;
import org.supurdueper.frc2023.commands.arm.MoveArmWithJoystick;
import org.supurdueper.frc2023.commands.elevator.ElevatorGoToPose;
import org.supurdueper.frc2023.commands.elevator.MoveElevatorWithJoystick;
import org.supurdueper.frc2023.commands.elevator.ResetElevatorPosition;
import org.supurdueper.frc2023.subsystems.Armavator.ArmavatorPose;
import org.supurdueper.frc2023.subsystems.Armavator.ArmavatorPose.ArmavatorPreset;
import org.supurdueper.frc2023.subsystems.arm.Arm;
import org.supurdueper.frc2023.subsystems.arm.ArmMotorIOSparkMax;
import org.supurdueper.frc2023.subsystems.drive.Drive;
import org.supurdueper.frc2023.subsystems.drive.GyroIO;
import org.supurdueper.frc2023.subsystems.drive.GyroIOPigeon2;
import org.supurdueper.frc2023.subsystems.drive.ModuleIO;
import org.supurdueper.frc2023.subsystems.drive.ModuleIOSim;
import org.supurdueper.frc2023.subsystems.drive.ModuleIOSparkMax;
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

    // *** DRIVER CONTROLS ***
    drive.setDefaultCommand(
        new DriveWithJoysticks(
            drive,
            () -> driver.getLeftY(),
            () -> driver.getLeftX(),
            () -> driver.getRightY(),
            () -> {
              return false;
            }));

    // For tuning
    operator.a().onTrue(armavatorGoToPose(ArmavatorPreset.midCube.getPose()));
    operator.b().onTrue(armavatorGoToPose(ArmavatorPreset.midCone.getPose()));
    operator.x().onTrue(armavatorGoToPose(ArmavatorPreset.highCube.getPose()));
    operator.y().onTrue(armavatorGoToPose(ArmavatorPreset.highCone.getPose()));
    operator.start().onTrue(new ResetElevatorPosition(elevator));
    elevator.setDefaultCommand(new MoveElevatorWithJoystick(elevator, operator::getRightY));
    arm.setDefaultCommand(new MoveArmWithJoystick(arm, operator::getLeftY));
  }

  /** Passes the autonomous command to the {@link Robot} class. */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public Command armavatorGoToPose(ArmavatorPose pose) {
    return new ElevatorGoToPose(elevator, pose.getElevatorProfileState())
        .alongWith(new ArmGoToPose(arm, pose.getArmProfileState()));
  }
}
