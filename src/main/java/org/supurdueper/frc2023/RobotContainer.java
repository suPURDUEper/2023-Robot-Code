// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.supurdueper.frc2023;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.littletonrobotics.frc2023.oi.HandheldOI;
import org.littletonrobotics.frc2023.oi.OISelector;
import org.littletonrobotics.frc2023.oi.OverrideOI;
import org.littletonrobotics.frc2023.util.Alert;
import org.littletonrobotics.frc2023.util.Alert.AlertType;
import org.littletonrobotics.frc2023.util.SparkMaxBurnManager;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.supurdueper.frc2023.Constants.Mode;
import org.supurdueper.frc2023.Subsystems.drive.Drive;
import org.supurdueper.frc2023.Subsystems.drive.GyroIO;
import org.supurdueper.frc2023.Subsystems.drive.GyroIOPigeon2;
import org.supurdueper.frc2023.Subsystems.drive.ModuleIO;
import org.supurdueper.frc2023.Subsystems.drive.ModuleIOSim;
import org.supurdueper.frc2023.Subsystems.drive.ModuleIOSparkMax;
import org.supurdueper.frc2023.commands.DriveWithJoysticks;

public class RobotContainer {

  /* Subsytems */
  private Drive drive;

  /* OI Objects */
  private OverrideOI overrideOI;
  private HandheldOI handheldOI;

  /* Choosers */
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  public RobotContainer() {
    /* Check If Flash Should Be Burned */
    SparkMaxBurnManager.update();
    // Instantiate active subsystems
    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case ROBOT_2023C:
          break;
        case ROBOT_2023P:
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOSparkMax(0),
                  new ModuleIOSparkMax(1),
                  new ModuleIOSparkMax(2),
                  new ModuleIOSparkMax(3));
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
    drive.setDefaultCommand(
        new DriveWithJoysticks(
            drive,
            () -> handheldOI.getLeftDriveX(),
            () -> handheldOI.getLeftDriveY(),
            () -> handheldOI.getRightDriveY(),
            () -> overrideOI.getRobotRelative()));

    // Set up auto routines
    autoChooser.addDefaultOption("Do Nothing", null);
    autoChooser.addDefaultOption(
        "Reset Odometry", new InstantCommand(() -> drive.setPose(new Pose2d())));

    // Alert if in tuning mode
    if (Constants.tuningMode) {
      new Alert("Tuning mode active, do not use in competition.", AlertType.INFO).set(true);
    }

    // Instantiate OI classes and bind buttons
    updateOI();
  }

  public void updateOI() {
    if (!OISelector.didJoysticksChange()) {
      return;
    }

    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    overrideOI = OISelector.findOverrideOI();
    handheldOI = OISelector.findHandheldOI();

    // *** DRIVER CONTROLS ***

    // *** OPERATOR CONTROLS ***
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
