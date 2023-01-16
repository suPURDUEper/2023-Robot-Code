package org.supurdueper.frc2023;

import edu.wpi.first.wpilibj.RobotBase;
import java.util.Map;
import org.littletonrobotics.frc2023.util.Alert;
import org.littletonrobotics.frc2023.util.Alert.AlertType;

public final class Constants {
  private static final RobotType robot = RobotType.ROBOT_2023P;
  public static final double loopPeriodSecs = 0.02;
  public static final boolean tuningMode = false;

  private static final Alert invalidRobotAlert =
      new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR);

  public static RobotType getRobot() {
    if (RobotBase.isReal()) {
      if (robot == RobotType.ROBOT_SIMBOT) { // Invalid robot selected
        invalidRobotAlert.set(true);
        return RobotType.ROBOT_2023C;
      } else {
        return robot;
      }
    } else {
      return robot;
    }
  }

  public static Mode getMode() {
    switch (getRobot()) {
      case ROBOT_2023C:
      case ROBOT_2023P:
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

      case ROBOT_SIMBOT:
        return Mode.SIM;

      default:
        return Mode.REAL;
    }
  }

  public static final Map<RobotType, String> logFolders =
      Map.of(RobotType.ROBOT_2023P, "/media/sda2/");

  public static enum RobotType {
    ROBOT_2023C,
    ROBOT_2023P,
    ROBOT_SIMBOT
  }

  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
