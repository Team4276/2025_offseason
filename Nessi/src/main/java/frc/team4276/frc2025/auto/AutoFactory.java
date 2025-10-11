package frc.team4276.frc2025.auto;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team4276.frc2025.RobotContainer;
import frc.team4276.frc2025.RobotState;
import frc.team4276.frc2025.field.FieldConstants;
import frc.team4276.frc2025.field.FieldConstants.ReefSide;
import frc.team4276.frc2025.subsystems.Superstructure.WantedSuperState;
import frc.team4276.frc2025.subsystems.SuperstructureConstants.ScoringSide;
import frc.team4276.frc2025.subsystems.elevator.ElevatorConstants.ElevatorPosition;
import frc.team4276.util.AllianceFlipUtil;
import frc.team4276.util.dashboard.Elastic;
import frc.team4276.util.dashboard.Elastic.Notification;
import frc.team4276.util.dashboard.Elastic.Notification.NotificationLevel;
import frc.team4276.util.path.ChoreoUtil;
import frc.team4276.util.path.PathUtil;

@SuppressWarnings("unused")
public class AutoFactory {
  private final RobotContainer robotContainer;

  public AutoFactory(final RobotContainer robotContainer) {
    this.robotContainer = robotContainer;
  }

  Command idle() {
    return resetPose(
        new Pose2d(
            RobotState.getInstance().getEstimatedPose().getTranslation(),
            AllianceFlipUtil.apply(Rotation2d.kZero)));
  }

  Command taxiCommand(boolean isProcessorSide) {
    Trajectory<SwerveSample> traj = ChoreoUtil.getChoreoTrajectory("t_WALL", !isProcessorSide);

    return resetPose(traj.getInitialPose(false).get());
  }

  Command poofsProcessorSide() {
    return resetPose(AllianceFlipUtil.apply(FieldConstants.blueProcessorSideStart))
        .andThen(driveAndScore(ReefSide.EF, WantedSuperState.SCORE_RIGHT_L2))
        .andThen(driveAndIntakeFromStation(FieldConstants.blueOutsideStationIntake))
        .andThen(driveAndAlgaePickup(ReefSide.CD, ScoringSide.LEFT))
        .andThen(driveAndScore(ReefSide.CD, WantedSuperState.SCORE_LEFT_L3))
        .andThen(driveAndIntakeFromStation(FieldConstants.blueOutsideStationIntake))
        .andThen(driveAndScore(ReefSide.CD, WantedSuperState.SCORE_RIGHT_L3))
        .andThen(driveAndIntakeFromStation(FieldConstants.blueOutsideStationIntake))
        .andThen(driveAndScore(ReefSide.CD, WantedSuperState.SCORE_LEFT_L2))
        .andThen(driveAndIntakeFromStation(FieldConstants.blueOutsideStationIntake))
        .andThen(driveAndScore(ReefSide.CD, WantedSuperState.SCORE_RIGHT_L2))
        .andThen(driveAndIntakeFromStation(FieldConstants.blueOutsideStationIntake));
  }

  Command poofsBargeSide() {
    return resetPose(
            PathUtil.mirrorLengthwise(
                AllianceFlipUtil.apply(FieldConstants.blueProcessorSideStart)))
        .andThen(driveAndScore(ReefSide.IJ, WantedSuperState.SCORE_LEFT_L2))
        .andThen(driveAndIntakeFromStation(FieldConstants.blueOutsideStationIntake))
        .andThen(driveAndAlgaePickup(ReefSide.KL, ScoringSide.RIGHT))
        .andThen(driveAndScore(ReefSide.KL, WantedSuperState.SCORE_RIGHT_L3))
        .andThen(driveAndIntakeFromStation(FieldConstants.blueOutsideStationIntake))
        .andThen(driveAndScore(ReefSide.KL, WantedSuperState.SCORE_LEFT_L3))
        .andThen(driveAndIntakeFromStation(FieldConstants.blueOutsideStationIntake))
        .andThen(driveAndScore(ReefSide.KL, WantedSuperState.SCORE_RIGHT_L2))
        .andThen(driveAndIntakeFromStation(FieldConstants.blueOutsideStationIntake))
        .andThen(driveAndScore(ReefSide.KL, WantedSuperState.SCORE_LEFT_L2))
        .andThen(driveAndIntakeFromStation(FieldConstants.blueOutsideStationIntake));
  }

  private Command resetPose(Pose2d pose) {
    return Commands.runOnce(() -> RobotState.getInstance().resetPose(pose));
  }

  private Command driveToPoint(Pose2d pose) {
    return Commands.runOnce(() -> robotContainer.getDrive().setAutoAlignPose(pose))
        .until(() -> robotContainer.getDrive().isAtAutoAlignPose());
  }

  private Command setState(WantedSuperState state) {
    return robotContainer.getSuperstructure().setStateCommand(state);
  }

  private Command waitForCoralIntake() {
    return Commands.waitUntil(() -> !robotContainer.getSuperstructure().hasCoral());
  }

  private Command driveAndScore(ReefSide reefSide, WantedSuperState state) {
    var side =
        (state == WantedSuperState.SCORE_LEFT_L1
                || state == WantedSuperState.SCORE_LEFT_L2
                || state == WantedSuperState.SCORE_LEFT_L3)
            ? ScoringSide.LEFT
            : ScoringSide.RIGHT;

    return driveToPoint(FieldConstants.getCoralScorePose(reefSide, side))
        .alongWith(setState(state))
        .andThen(waitForCoralIntake().raceWith(Commands.waitSeconds(1.0)))
        .andThen(Commands.waitSeconds(0.25));
  }

  private Command driveAndIntakeFromStation(Pose2d intakePose) {
    return (driveToPoint(intakePose)
            .alongWith(setState(WantedSuperState.INTAKE_CORAL))
            .andThen(Commands.waitSeconds(3.0)))
        .raceWith(waitForCoralIntake());
  }

  private Command driveAndAlgaePickup(ReefSide reefSide, ScoringSide side) {
    return driveToPoint(FieldConstants.getClearReefPose(reefSide, side))
        .andThen(setState(WantedSuperState.REEF_ALGAE))
        .andThen(
            Commands.waitUntil(
                () ->
                    robotContainer.getElevator().getWantedElevatorPose()
                        == ElevatorPosition.ALGAE_CHOP))
        .andThen(Commands.waitUntil(() -> robotContainer.getDrive().isAtAutoAlignPose()))
        .andThen(
            Commands.runOnce(
                () ->
                    robotContainer
                        .getDrive()
                        .setHeadingAlignRotation(
                            FieldConstants.getClearReefPose(reefSide, side)
                                .getRotation()
                                .plus(
                                    Rotation2d.fromDegrees(
                                        side == ScoringSide.LEFT ? -90.0 : 90.0)))));
  }

  /**
   * Returns whether robot has crossed x boundary, accounting for alliance flip
   *
   * @param xPosition X position coordinate on blue side of field.
   * @param towardsCenterline Whether to wait until passed x coordinate towards center line or away
   *     from center line
   */
  private boolean xCrossed(double xPosition, boolean towardsCenterline) {
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
  private Command waitUntilXCrossed(double xPosition, boolean towardsCenterline) {
    return Commands.waitUntil(() -> xCrossed(xPosition, towardsCenterline));
  }

  /**
   * Returns whether robot has crossed y boundary, accounting for alliance flip
   *
   * @param yPosition Y position coordinate on blue side of field.
   * @param towardsCenterline Whether to wait until passed y coordinate towards center line or away
   *     from center line
   */
  private boolean yCrossed(double yPosition, boolean towardsCenterline) {
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
  private Command waitUntilYCrossed(double yPosition, boolean towardsCenterline) {
    return Commands.waitUntil(() -> yCrossed(yPosition, towardsCenterline));
  }

  private Command printCommand(String text) {
    return Commands.runOnce(() -> System.out.println(text));
  }

  private Command notificationCommand(String notification) {
    return notificationCommand(
        new Notification(NotificationLevel.INFO, "Auto Action", notification, 3000));
  }

  private Command notificationCommand(Notification notification) { // Jank but gud enough for now
    return Commands.runOnce(() -> Elastic.sendNotification(notification));
  }
}
