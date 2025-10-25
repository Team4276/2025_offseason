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
import frc.team4276.util.dashboard.ElasticUI;
import frc.team4276.util.path.ChoreoUtil;
import java.util.List;
import java.util.function.Supplier;

@SuppressWarnings("unused")
public class AutoFactory {
  private RobotContainer robotContainer;

  private final double distanceToScoreElevatorRaise = 2.0;

  public AutoFactory(RobotContainer robotContainer) {
    this.robotContainer = robotContainer;
  }

  Command idle() {
    ElasticUI.putAutoPath(() -> List.of());

    return resetPose(
        new Pose2d(
            RobotState.getInstance().getEstimatedPose().getTranslation(),
            AllianceFlipUtil.apply(Rotation2d.kZero)));
  }

  void autoEnd() {
    robotContainer.getSuperstructure().setWantedSuperState(WantedSuperState.STOW);
  }

  Command taxiCommand(boolean isBargeSide) {
    Trajectory<SwerveSample> traj = ChoreoUtil.getChoreoTrajectory("t_WALL", isBargeSide);

    ElasticUI.putAutoTrajectory(traj);

    return resetPose(traj.getInitialPose(false).get()).andThen(driveTrajectory(traj));
  }

  Command taxiMidCommand(boolean isBargeSide) {
    Trajectory<SwerveSample> traj = ChoreoUtil.getChoreoTrajectory("c_st_sc_G", isBargeSide);

    ElasticUI.putAutoTrajectory(traj);

    return resetPose(traj.getInitialPose(false).get())
        .andThen(
            driveAndScore(
                traj,
                ReefSide.GH,
                isBargeSide ? WantedSuperState.SCORE_LEFT_L1 : WantedSuperState.SCORE_RIGHT_L1));
  }

  Command EBA() {
    var startPose = FieldConstants.flippablePose(FieldConstants.blueProcessorSideStart, false);
    var intakePose = FieldConstants.flippablePose(FieldConstants.blueOutsideStationIntake, false);

    return resetPose(startPose)
        .andThen(driveAndScore(ReefSide.EF, WantedSuperState.SCORE_RIGHT_L2))
        .andThen(driveAndIntakeFromStation(intakePose))
        .andThen(driveAndScore(ReefSide.AB, WantedSuperState.SCORE_RIGHT_L2))
        .andThen(driveAndIntakeFromStation(intakePose))
        .andThen(driveAndScore(ReefSide.AB, WantedSuperState.SCORE_LEFT_L2))
        .andThen(driveAndIntakeFromStation(intakePose));
  }

  Command JAB() {
    var startPose = FieldConstants.flippablePose(FieldConstants.blueProcessorSideStart, true);
    var intakePose = FieldConstants.flippablePose(FieldConstants.blueOutsideStationIntake, true);

    return resetPose(startPose)
        .andThen(driveAndScore(ReefSide.IJ, WantedSuperState.SCORE_LEFT_L2))
        .andThen(driveAndIntakeFromStation(intakePose))
        .andThen(driveAndScore(ReefSide.AB, WantedSuperState.SCORE_LEFT_L2))
        .andThen(driveAndIntakeFromStation(intakePose))
        .andThen(driveAndScore(ReefSide.AB, WantedSuperState.SCORE_RIGHT_L2))
        .andThen(driveAndIntakeFromStation(intakePose));
  }

  Command FEDC() {
    var startPose = FieldConstants.flippablePose(FieldConstants.blueProcessorSideStart, false);
    var intakePose = FieldConstants.flippablePose(FieldConstants.blueOutsideStationIntake, false);

    return resetPose(startPose)
        .andThen(driveAndScore(ReefSide.EF, WantedSuperState.SCORE_LEFT_L2))
        .andThen(driveAndIntakeFromStation(intakePose))
        .andThen(driveAndScore(ReefSide.EF, WantedSuperState.SCORE_RIGHT_L2))
        .andThen(driveAndIntakeFromStation(intakePose))
        .andThen(driveAndScore(ReefSide.CD, WantedSuperState.SCORE_LEFT_L1))
        .andThen(driveAndIntakeFromStation(intakePose))
        .andThen(driveAndScore(ReefSide.CD, WantedSuperState.SCORE_RIGHT_L1))
        .andThen(driveAndIntakeFromStation(intakePose));
  }

  Command IJKL() {
    var startPose = FieldConstants.flippablePose(FieldConstants.blueProcessorSideStart, true);
    var intakePose = FieldConstants.flippablePose(FieldConstants.blueOutsideStationIntake, true);

    return resetPose(startPose)
        .andThen(driveAndScore(ReefSide.IJ, WantedSuperState.SCORE_RIGHT_L2))
        .andThen(driveAndIntakeFromStation(intakePose))
        .andThen(driveAndScore(ReefSide.IJ, WantedSuperState.SCORE_LEFT_L2))
        .andThen(driveAndIntakeFromStation(intakePose))
        .andThen(driveAndScore(ReefSide.KL, WantedSuperState.SCORE_RIGHT_L1))
        .andThen(driveAndIntakeFromStation(intakePose))
        .andThen(driveAndScore(ReefSide.KL, WantedSuperState.SCORE_LEFT_L1))
        .andThen(driveAndIntakeFromStation(intakePose));
  }

  Command poofsProcessorSide() {
    var startPose = FieldConstants.flippablePose(FieldConstants.blueProcessorSideStart, false);
    var intakePose = FieldConstants.flippablePose(FieldConstants.blueOutsideStationIntake, false);

    return resetPose(startPose)
        .andThen(driveAndScore(ReefSide.EF, WantedSuperState.SCORE_RIGHT_L2))
        .andThen(driveAndIntakeFromStation(intakePose))
        .andThen(driveAndAlgaePickup(ReefSide.CD, ScoringSide.LEFT))
        .andThen(algaeSwing(ReefSide.CD, ScoringSide.LEFT))
        .andThen(driveAndScore(ReefSide.CD, WantedSuperState.SCORE_LEFT_L3))
        .andThen(driveAndIntakeFromStation(intakePose))
        .andThen(driveAndScore(ReefSide.CD, WantedSuperState.SCORE_RIGHT_L3))
        .andThen(driveAndIntakeFromStation(intakePose))
        .andThen(driveAndScore(ReefSide.CD, WantedSuperState.SCORE_LEFT_L2))
        .andThen(driveAndIntakeFromStation(intakePose))
        .andThen(driveAndScore(ReefSide.CD, WantedSuperState.SCORE_RIGHT_L2))
        .andThen(driveAndIntakeFromStation(intakePose));
  }

  Command poofsBargeSide() {
    var startPose = FieldConstants.flippablePose(FieldConstants.blueProcessorSideStart, true);
    var intakePose = FieldConstants.flippablePose(FieldConstants.blueOutsideStationIntake, true);

    return resetPose(startPose)
        .andThen(driveAndScore(ReefSide.IJ, WantedSuperState.SCORE_LEFT_L2))
        .andThen(driveAndIntakeFromStation(intakePose))
        .andThen(driveAndAlgaePickup(ReefSide.KL, ScoringSide.RIGHT))
        .andThen(algaeSwing(ReefSide.KL, ScoringSide.RIGHT))
        .andThen(driveAndScore(ReefSide.KL, WantedSuperState.SCORE_RIGHT_L3))
        .andThen(driveAndIntakeFromStation(intakePose))
        .andThen(driveAndScore(ReefSide.KL, WantedSuperState.SCORE_LEFT_L3))
        .andThen(driveAndIntakeFromStation(intakePose))
        .andThen(driveAndScore(ReefSide.KL, WantedSuperState.SCORE_RIGHT_L2))
        .andThen(driveAndIntakeFromStation(intakePose))
        .andThen(driveAndScore(ReefSide.KL, WantedSuperState.SCORE_LEFT_L2))
        .andThen(driveAndIntakeFromStation(intakePose));
  }

  Command jitbProcessorSide() {
    Trajectory<SwerveSample> traj = ChoreoUtil.getChoreoTrajectory("jitb_start");
    var intakePose = FieldConstants.flippablePose(FieldConstants.blueInsideStationIntake, false);

    return resetPose(traj.getInitialPose(false).get())
        .andThen(driveAndScore(traj, ReefSide.AB, WantedSuperState.SCORE_RIGHT_L2))
        .andThen(driveAndIntakeFromStation(intakePose))
        .andThen(driveAndScore(ReefSide.AB, WantedSuperState.SCORE_LEFT_L2))
        .andThen(driveAndAlgaePickup(ReefSide.AB, ScoringSide.RIGHT))
        .andThen(driveAndIntakeFromStation(intakePose))
        .andThen(driveAndScore(ReefSide.AB, WantedSuperState.SCORE_RIGHT_L3))
        .andThen(driveAndIntakeFromStation(intakePose))
        .andThen(driveAndScore(ReefSide.AB, WantedSuperState.SCORE_LEFT_L3))
        .andThen(driveAndIntakeFromStation(intakePose));
  }

  Command jitbBargeSide() {
    Trajectory<SwerveSample> traj = ChoreoUtil.getChoreoTrajectory("jitb_start", true);
    var intakePose = FieldConstants.flippablePose(FieldConstants.blueInsideStationIntake, true);

    return resetPose(traj.getInitialPose(false).get())
        .andThen(driveAndScore(traj, ReefSide.AB, WantedSuperState.SCORE_LEFT_L2))
        .andThen(driveAndIntakeFromStation(intakePose))
        .andThen(driveAndScore(ReefSide.AB, WantedSuperState.SCORE_RIGHT_L2))
        .andThen(driveAndAlgaePickup(ReefSide.AB, ScoringSide.RIGHT))
        .andThen(driveAndIntakeFromStation(intakePose))
        .andThen(driveAndScore(ReefSide.AB, WantedSuperState.SCORE_LEFT_L3))
        .andThen(driveAndIntakeFromStation(intakePose))
        .andThen(driveAndScore(ReefSide.AB, WantedSuperState.SCORE_RIGHT_L3))
        .andThen(driveAndIntakeFromStation(intakePose));
  }

  private Command resetPose(Pose2d pose) {
    return Commands.runOnce(() -> RobotState.getInstance().resetPose(pose));
  }

  private Command driveTrajectory(Trajectory<SwerveSample> traj) {
    return Commands.runOnce(() -> robotContainer.getDrive().setChoreoTrajectory(traj))
        .andThen(Commands.waitUntil(() -> robotContainer.getDrive().isTrajectoryFinished()));
  }

  private Command driveToPoint(Pose2d pose) {
    return driveToPoint(() -> pose);
  }

  private Command driveToPoint(Supplier<Pose2d> pose) {
    return Commands.run(() -> robotContainer.getDrive().setAutoAlignPose(pose.get()))
        .until(() -> robotContainer.getDrive().isAtAutoAlignPose());
  }

  private Command setState(WantedSuperState state) {
    return robotContainer.getSuperstructure().setStateCommand(state);
  }

  private Command waitForCoralScore() {
    return Commands.waitUntil(() -> !robotContainer.getSuperstructure().hasCoral());
  }

  private Command waitForCoralIntake() {
    return Commands.waitUntil(() -> robotContainer.getSuperstructure().hasCoral());
  }

  private Command waitForRaise(Pose2d endPose, double distanceToElevatorRaise) {
    return Commands.waitUntil(
        () ->
            robotContainer
                .getDrive()
                .isAtTranslation(endPose.getTranslation(), distanceToElevatorRaise));
  }

  private Command driveAndScore(ReefSide reefSide, WantedSuperState state) {
    return driveAndScore(reefSide, state, distanceToScoreElevatorRaise);
  }

  private Command driveAndScore(
      ReefSide reefSide, WantedSuperState state, double distanceToElevatorRaise) {
    var side =
        (state == WantedSuperState.SCORE_LEFT_L1
                || state == WantedSuperState.SCORE_LEFT_L2
                || state == WantedSuperState.SCORE_LEFT_L3)
            ? ScoringSide.LEFT
            : ScoringSide.RIGHT;

    return driveToPoint(
            () -> robotContainer.getSuperstructure().getAutoAlignCoralScorePose(reefSide, side))
        .alongWith(
            waitForRaise(FieldConstants.getCoralScorePose(reefSide, side), distanceToElevatorRaise)
                .andThen(setState(state)))
        .andThen(waitForCoralScore().raceWith(Commands.waitSeconds(1.0)))
        .andThen(Commands.waitSeconds(0.25));
  }

  private Command driveAndScore(
      Trajectory<SwerveSample> traj, ReefSide reefSide, WantedSuperState state) {
    return driveAndScore(traj, reefSide, state, distanceToScoreElevatorRaise);
  }

  private Command driveAndScore(
      Trajectory<SwerveSample> traj,
      ReefSide reefSide,
      WantedSuperState state,
      double distanceToElevatorRaise) {
    var side =
        (state == WantedSuperState.SCORE_LEFT_L1
                || state == WantedSuperState.SCORE_LEFT_L2
                || state == WantedSuperState.SCORE_LEFT_L3)
            ? ScoringSide.LEFT
            : ScoringSide.RIGHT;

    return driveTrajectory(traj)
        .andThen(
            driveToPoint(
                () ->
                    robotContainer.getSuperstructure().getAutoAlignCoralScorePose(reefSide, side)))
        .alongWith(
            waitForRaise(FieldConstants.getCoralScorePose(reefSide, side), distanceToElevatorRaise)
                .andThen(setState(state)))
        .andThen(waitForCoralScore().raceWith(Commands.waitSeconds(1.0)))
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
                            == ElevatorPosition.ALGAE_CHOP
                        && robotContainer.getElevator().atGoal()))
        .andThen(Commands.waitUntil(() -> robotContainer.getDrive().isAtAutoAlignPose()));
  }

  private Command algaeSwing(ReefSide reefSide, ScoringSide side) {
    return Commands.runOnce(
            () ->
                robotContainer
                    .getDrive()
                    .setHeadingAlignRotation(
                        FieldConstants.getClearReefPose(reefSide, side)
                            .getRotation()
                            .plus(Rotation2d.fromDegrees(side == ScoringSide.LEFT ? -90.0 : 90.0))))
        .andThen(Commands.waitUntil(() -> robotContainer.getDrive().isHeadingAligned()));
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
