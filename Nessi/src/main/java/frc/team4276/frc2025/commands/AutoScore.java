package frc.team4276.frc2025.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team4276.frc2025.RobotState;
import frc.team4276.frc2025.ScoringHelper;
import frc.team4276.frc2025.field.FieldConstants.Reef;
import frc.team4276.frc2025.subsystems.drive.Drive;
import frc.team4276.frc2025.subsystems.superstructure.Superstructure;
import frc.team4276.util.dashboard.LoggedTunableNumber;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class AutoScore {
  private static final LoggedTunableNumber reefAlignThreshold =
      new LoggedTunableNumber("AutoScore/ReefAlignThreshold", 1.0);
  private static final LoggedTunableNumber reefNudgeThreshold =
      new LoggedTunableNumber("AutoScore/ReefNudgeThreshold", 0.1);

  private static boolean cancelTxTy = false;

  private static boolean proceedScoring = false;

  private static boolean proceedScoring() {
    return proceedScoring;
  }

  public static Command selectAndScoreCommand(
      Superstructure superstructure, Superstructure.Goal goal) {
    return Commands.runOnce(
        () -> {
          superstructure.selectAutoScoreGoal(goal);
          proceedScoring = true;
        });
  }

  public static Command coralAlignCommand(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier headingSupplier,
      boolean isLeft,
      Superstructure superstructure) {
    Supplier<Pose2d> robotPose = RobotState.getInstance()::getReefPose;

    Supplier<Optional<Reef>> goal =
        () ->
            RobotState.getInstance()
                .getPriorityReefTag()
                .map(tag -> getReefFromSide(getSideFromTagId(tag), isLeft))
                .or(
                    () ->
                        Optional.of(
                            getReefFromSide(
                                getSideFromTagId(RobotState.getInstance().getLastPriorityTag()),
                                isLeft)));

    return Commands.runOnce(() -> proceedScoring = false)
        .andThen(
            DriveCommands.joystickDrive(drive, xSupplier, ySupplier, headingSupplier)
                .until(
                    () ->
                        RobotState.getInstance()
                            .getPriorityReefTag()
                            .map(tag -> getReefFromSide(getSideFromTagId(tag), isLeft))
                            .isPresent()))
        .andThen(
            new DriveToPose(drive, () -> goal.get().get().getAlign(), robotPose)
                .until(
                    () ->
                        // inTolerance(
                        // goal.get().get(),
                        // goal.get().get().getAlign(),
                        // reefNudgeThreshold.get())
                        // &&
                        proceedScoring()))
        .andThen(
            new DriveToPose(drive, () -> goal.get().get().getScore(), robotPose)
                .alongWith(
                    // superstructure.setGoalCommand(level),
                    superstructure.autoScoreCommand(),
                    Commands.waitUntil(
                            () ->
                                superstructure.atGoal()
                                    && superstructure.getGoal() != Superstructure.Goal.STOW
                                    && DriveToPose.atGoal())
                        .andThen(superstructure.scoreCommand(false))
                        .andThen(Commands.waitSeconds(0.5))))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public static Command coralAlignCommand(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Superstructure superstructure,
      ScoringHelper scoringHelper) {

    Supplier<Pose2d> alignPose = scoringHelper.getSelectedReef()::getAlign;
    Supplier<Pose2d> scorePose = scoringHelper.getSelectedReef()::getScore;

    return Commands.sequence(
        Commands.waitUntil(
                () ->
                    inTolerance(
                        scoringHelper.getSelectedReef(), alignPose.get(), reefAlignThreshold.get()))
            .deadlineFor(
                DriveCommands.joystickDriveAtHeading(
                    drive, xSupplier, ySupplier, () -> scorePose.get().getRotation())),
        new DriveToPose(
                drive,
                alignPose,
                () -> getRobotPose(scoringHelper.getSelectedReef(), alignPose.get()))
            .until(
                () ->
                    inTolerance(
                        scoringHelper.getSelectedReef(),
                        scorePose.get(),
                        reefNudgeThreshold.get())),
        new DriveToPose(
                drive,
                scorePose,
                () -> getRobotPose(scoringHelper.getSelectedReef(), scorePose.get()))
            .alongWith(superstructure.setGoalCommand(scoringHelper.getSuperstructureGoal())));
  }

  public static Command bargeScoreCommand() {
    return Commands.none();
  }

  private static int getSideFromTagId(int id) {
    return switch (id) {
      case 6 -> 5;
      case 7 -> 0;
      case 8 -> 1;
      case 9 -> 2;
      case 10 -> 3;
      case 11 -> 4;

      case 17 -> 1;
      case 18 -> 0;
      case 19 -> 5;
      case 20 -> 4;
      case 21 -> 3;
      case 22 -> 2;

      default -> -1;
    };
  }

  private static Reef getReefFromSide(int side, boolean isLeft) {
    if (side > 1 && side < 5) {
      return Reef.values()[(side * 2) + (isLeft ? 1 : 0)];

    } else if (side > 0) {
      return Reef.values()[(side * 2) + (isLeft ? 0 : 1)];

    } else {
      return Reef.A;
    }
  }

  private static Pose2d getRobotPose(Reef reef, Pose2d finalPose) {
    return cancelTxTy
        ? RobotState.getInstance().getEstimatedPose()
        : RobotState.getInstance().getReefPose(reef.ordinal() / 2, finalPose);
  }

  private static boolean inTolerance(Reef reef, Pose2d pose, double tol) {
    return pose.getTranslation().getDistance(getRobotPose(reef, pose).getTranslation()) < tol;
  }
}
