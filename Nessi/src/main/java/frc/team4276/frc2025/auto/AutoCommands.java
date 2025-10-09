package frc.team4276.frc2025.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team4276.frc2025.RobotState;
import frc.team4276.frc2025.field.FieldConstants;
import frc.team4276.frc2025.subsystems.Superstructure;
import frc.team4276.util.AllianceFlipUtil;
import frc.team4276.util.dashboard.Elastic;
import frc.team4276.util.dashboard.Elastic.Notification;
import frc.team4276.util.dashboard.Elastic.Notification.NotificationLevel;

public class AutoCommands {
  private AutoCommands() {}

  public static final double scoreWaitTime = 0.5;
  public static final double intakeWaitTime = 1.5;

  public static Command resetPose(Pose2d pose) {
    return Commands.runOnce(() -> RobotState.getInstance().resetPose(pose));
  }

  /**
   * Returns whether robot has crossed x boundary, accounting for alliance flip
   *
   * @param xPosition X position coordinate on blue side of field.
   * @param towardsCenterline Whether to wait until passed x coordinate towards center line or away
   *     from center line
   */
  public static boolean xCrossed(double xPosition, boolean towardsCenterline) {
    Pose2d robotPose = RobotState.getInstance().getTrajectorySetpoint();
    if (AllianceFlipUtil.shouldFlip()) {
      if (towardsCenterline) {
        return robotPose.getX() < FieldConstants.fieldLength - xPosition;
      } else {
        return robotPose.getX() > FieldConstants.fieldLength - xPosition;
      }
    } else {
      if (towardsCenterline) {
        return robotPose.getX() > xPosition;
      } else {
        return robotPose.getX() < xPosition;
      }
    }
  }

  /** Command that waits for x boundary to be crossed. See {@link #xCrossed(double, boolean)} */
  public static Command waitUntilXCrossed(double xPosition, boolean towardsCenterline) {
    return Commands.waitUntil(() -> xCrossed(xPosition, towardsCenterline));
  }

  /**
   * Returns whether robot has crossed y boundary, accounting for alliance flip
   *
   * @param yPosition Y position coordinate on blue side of field.
   * @param towardsCenterline Whether to wait until passed y coordinate towards center line or away
   *     from center line
   */
  public static boolean yCrossed(double yPosition, boolean towardsCenterline) {
    Pose2d robotPose = RobotState.getInstance().getTrajectorySetpoint();
    if (AllianceFlipUtil.shouldFlip()) {
      if (towardsCenterline) {
        return robotPose.getY() < FieldConstants.fieldWidth - yPosition;
      } else {
        return robotPose.getY() > FieldConstants.fieldWidth - yPosition;
      }
    } else {
      if (towardsCenterline) {
        return robotPose.getY() > yPosition;
      } else {
        return robotPose.getY() < yPosition;
      }
    }
  }

  /** Command that waits for y boundary to be crossed. See {@link #yCrossed(double, boolean)} */
  public static Command waitUntilYCrossed(double yPosition, boolean towardsCenterline) {
    return Commands.waitUntil(() -> yCrossed(yPosition, towardsCenterline));
  }

  public static Command printCommand(String text) {
    return Commands.runOnce(() -> System.out.println(text));
  }

  public static Command notificationCommand(String notification) {
    return notificationCommand(
        new Notification(NotificationLevel.INFO, "Auto Action", notification, 3000));
  }

  public static Command notificationCommand(
      Notification notification) { // Jank but gud enough for now
    return Commands.runOnce(() -> Elastic.sendNotification(notification));
  }

  public static Command scoreCommand(Superstructure superstructure) {
    return scoreCommand(superstructure, false);
  }

  public static Command scoreCommand(Superstructure superstructure, boolean isLeftL1) {
    return Commands.none();
    // superstructure
    //     .scoreCommand(isLeftL1)
    //     .alongWith(notificationCommand("Scoring"))
    //     .withTimeout(scoreWaitTime)
    //     .withName("Score");
  }
}
