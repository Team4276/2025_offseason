package frc.team4276.frc2025.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team4276.frc2025.RobotState;
import frc.team4276.frc2025.field.FieldConstants.Reef;
import frc.team4276.frc2025.field.FieldConstants.ReefSide;
import frc.team4276.frc2025.subsystems.Superstructure;
import frc.team4276.frc2025.subsystems.drive.Drive;
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
      Superstructure superstructure, Superstructure.WantedSuperState goal) {
    return Commands.runOnce(
        () -> {
          // superstructure.selectAutoScoreGoal(goal);
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

    // Supplier<Optional<Reef>> goal = () -> RobotState.getInstance()
    // .getPriorityReefTag()
    // .map(tag -> getSideFromTagId(tag).get())
    // .or(
    // () -> getSideFromTagId(RobotState.getInstance().getLastPriorityTag()));

    return Commands.none();
    // return Commands.runOnce(() -> proceedScoring = false)
    // .andThen(
    // DriveCommands.joystickDrive(drive, xSupplier, ySupplier, headingSupplier)
    // .until(
    // () ->
    // RobotState.getInstance()
    // .getPriorityReefTag()
    // .map(tag -> getReefFromSide(getSideFromTagId(tag), isLeft))
    // .isPresent()))
    // .andThen(
    // new DriveToPose(drive, () -> goal.get().get().getAlign(), robotPose)
    // .until(() -> proceedScoring()))
    // .andThen(
    // new DriveToPose(drive, () -> goal.get().get().getScore(), robotPose)
    // .alongWith(
    // Commands.waitUntil(
    // () ->
    // // superstructure.atGoal()
    // // &&
    // superstructure.getCurrentSuperState()
    // != Superstructure.CurrentSuperState.STOW
    // && DriveToPose.atGoal())
    // .andThen(Commands.waitSeconds(0.5))))
    // .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public static Command bargeScoreCommand() {
    return Commands.none();
  }

  private static Optional<ReefSide> getSideFromTagId(int id) {
    return switch (id) {
      case 6 -> Optional.of(ReefSide.KL);
      case 7 -> Optional.of(ReefSide.AB);
      case 8 -> Optional.of(ReefSide.CD);
      case 9 -> Optional.of(ReefSide.EF);
      case 10 -> Optional.of(ReefSide.GH);
      case 11 -> Optional.of(ReefSide.IJ);

      case 17 -> Optional.of(ReefSide.CD);
      case 18 -> Optional.of(ReefSide.AB);
      case 19 -> Optional.of(ReefSide.KL);
      case 20 -> Optional.of(ReefSide.IJ);
      case 21 -> Optional.of(ReefSide.GH);
      case 22 -> Optional.of(ReefSide.EF);

      default -> Optional.empty();
    };
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
