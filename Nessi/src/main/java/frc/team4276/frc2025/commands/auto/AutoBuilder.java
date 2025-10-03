package frc.team4276.frc2025.commands.auto;

import static frc.team4276.frc2025.commands.auto.AutoCommands.*;
import static frc.team4276.util.path.PPUtil.*;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team4276.frc2025.AutoSelector;
import frc.team4276.frc2025.AutoSelector.AutoQuestionResponse;
import frc.team4276.frc2025.subsystems.superstructure.Superstructure;
import frc.team4276.frc2025.subsystems.superstructure.Superstructure.CurrentSuperState;
import frc.team4276.frc2025.subsystems.superstructure.Superstructure.WantedSuperState;
import java.util.ArrayList;
import java.util.List;

public class AutoBuilder {
  private final Superstructure superstructure; // TODO: add drive to pose auto
  private final AutoSelector autoSelector;

  public AutoBuilder(Superstructure superstructure, AutoSelector autoSelector) {
    this.superstructure = superstructure;
    this.autoSelector = autoSelector;
  }

  public Command testTraj(String name) {
    // var traj = getPathPlannerTrajectoryFromChoreo(name);

    return Commands.none();
    // resetPose(traj.getInitialPose()).andThen(new DriveTrajectory(drive, traj));
  }

  public Command taxiAuto(String name) {
    // Check if need to flip paths to barge side
    // boolean mirrorLengthwise = autoSelector.getResponses().get(0) == AutoQuestionResponse.NO;

    // var traj = getPathPlannerTrajectoryFromChoreo(name, mirrorLengthwise);

    return Commands.none();
    // Commands.sequence(
    //     resetPose(traj.getInitialPose()),
    //     Commands.waitSeconds(autoSelector.getDelayInput()),
    //     new DriveTrajectory(drive, traj));
  }

  public Command sandyEggosAuto(
      List<AutoQuestionResponse> reefs, List<AutoQuestionResponse> levels) {

    if (reefs.isEmpty() || levels.isEmpty()) {
      CommandScheduler.getInstance()
          .schedule(notificationCommand("Error invalid Coral Auto List is Empty"));

      return Commands.none();
    }

    // Check if need to flip paths to barge side
    boolean mirrorLengthwise = autoSelector.getResponses().get(0) == AutoQuestionResponse.NO;

    var scoringCommand = new SequentialCommandGroup();

    String station =
        autoSelector.getResponses().get(2) == AutoQuestionResponse.YES ? "CLOSE" : "FAR";

    PathPlannerTrajectory traj1;
    PathPlannerTrajectory traj2;

    if (autoSelector.getResponses().get(1) == AutoQuestionResponse.YES) {
      traj1 =
          getPathPlannerTrajectoryFromChoreo(
              "c_sw_sc_" + reefs.get(0).toString(), mirrorLengthwise);

    } else {
      traj1 =
          getPathPlannerTrajectoryFromChoreo(
              "c_st_sc_" + reefs.get(0).toString(), mirrorLengthwise);
    }

    if (autoSelector.getResponses().get(2) == AutoQuestionResponse.YES) {
      traj2 =
          getPathPlannerTrajectoryFromChoreo("c_kn_" + reefs.get(0).toString(), mirrorLengthwise);

    } else {
      traj2 =
          getPathPlannerTrajectoryFromChoreo(
              "c_sc_" + station + "_" + reefs.get(0).toString(), mirrorLengthwise, 1);
    }

    for (int i = 0; i < reefs.size(); i++) {
      var scTraj =
          i == 0
              ? traj1
              : getPathPlannerTrajectoryFromChoreo(
                  "c_sc_" + station + "_" + reefs.get(i).toString(), mirrorLengthwise, 0);
      var intTraj =
          i == 0
              ? traj2
              : getPathPlannerTrajectoryFromChoreo(
                  "c_sc_" + station + "_" + reefs.get(i).toString(), mirrorLengthwise, 1);

      scoringCommand.addCommands(shrimpleCoral(scTraj, intTraj, toGoal(levels.get(i))));
    }

    return Commands.sequence(
        resetPose(traj1.getInitialPose()),
        Commands.waitSeconds(autoSelector.getDelayInput()),
        scoringCommand);
  }

  public Command sandyEggosAuto() {
    List<AutoQuestionResponse> reefs = new ArrayList<>();

    for (int i = 5; i < 8; i++) {
      int ordinal = autoSelector.getResponses().get(i).ordinal() - AutoQuestionResponse.A.ordinal();
      if (ordinal >= 0 && ordinal < 12) {
        reefs.add(autoSelector.getResponses().get(i));
      }
    }

    return sandyEggosAuto(reefs, reefsToLevels(reefs));
  }

  public Command speedyIntakeAuto(
      List<AutoQuestionResponse> reefs, List<AutoQuestionResponse> levels) {

    if (reefs.isEmpty() || levels.isEmpty()) {
      CommandScheduler.getInstance()
          .schedule(notificationCommand("Error invalid Coral Auto List is Empty"));

      return Commands.none();
    }

    // Check if need to flip paths to barge side
    boolean mirrorLengthwise = autoSelector.getResponses().get(0) == AutoQuestionResponse.NO;

    var scoringCommand = new SequentialCommandGroup();

    var traj1 =
        getPathPlannerTrajectoryFromChoreo("c_st_sc_" + reefs.get(0).toString(), mirrorLengthwise);

    // scoringCommand.addCommands(
    //     Commands.sequence(
    //             new DriveTrajectory(drive, traj1),
    //             Commands.waitUntil(
    //                 () ->
    //                     superstructure.atGoal()
    //                         && superstructure.getGoal() != WantedSuperState.STOW),
    //             scoreCommand(superstructure))
    //         .deadlineFor(
    //             Commands.waitSeconds(traj1.getTotalTimeSeconds() - 0.75)
    //                 .andThen(superstructure.setGoalCommand(toGoal(levels.get(0))))));

    for (int i = 1; i < reefs.size(); i++) {
      var traj =
          getPathPlannerTrajectoryFromChoreo(
              "c_sp_FAR_" + reefs.get(i).toString(), mirrorLengthwise);

      scoringCommand.addCommands(speedyCoral(traj, toGoal(levels.get(i))));
    }

    return Commands.sequence(
        resetPose(traj1.getInitialPose()),
        Commands.waitSeconds(autoSelector.getDelayInput()),
        scoringCommand);
  }

  private Command speedyCoral(PathPlannerTrajectory traj, WantedSuperState goal) {
    return Commands.none();
    // Commands.sequence(
    //         new DriveTrajectory(drive, traj)
    //             .andThen(
    //                 Commands.waitUntil(
    //                     () ->
    //                         superstructure.atGoal()
    //                             && superstructure.getGoal() != WantedSuperState.STOW),
    //                 scoreCommand(superstructure)))
    //     .deadlineFor(
    //         superstructure
    //             .setGoalCommand(WantedSuperState.INTAKE_CORAL)
    //             .withDeadline(
    //                 Commands.either(
    //                     Commands.waitSeconds(1.0),
    //                     Commands.waitUntil(() -> superstructure.hasCoral()),
    //                     () -> Constants.isSim))
    //             .andThen(
    //                 Commands.waitSeconds(traj.getTotalTimeSeconds() - 0.75)
    //                     .andThen(superstructure.setGoalCommand(goal))));
  }

  public Command shrimpleOcrAuto(
      List<AutoQuestionResponse> reefs, List<AutoQuestionResponse> levels) {

    if (reefs.isEmpty() || levels.isEmpty()) {
      CommandScheduler.getInstance()
          .schedule(notificationCommand("Error invalid Coral Auto List is Empty"));

      return Commands.none();
    }

    // Check if need to flip paths to barge side
    boolean mirrorLengthwise = autoSelector.getResponses().get(0) == AutoQuestionResponse.NO;

    var scoringCommand = new SequentialCommandGroup();

    var traj1 =
        getPathPlannerTrajectoryFromChoreo("c_st_sc_" + reefs.get(0).toString(), mirrorLengthwise);

    for (int i = 0; i < reefs.size(); i++) {
      var scTraj =
          i == 0
              ? traj1
              : getPathPlannerTrajectoryFromChoreo(
                  "c_sc_FAR_" + reefs.get(i).toString(), mirrorLengthwise, 0);
      var intTraj =
          getPathPlannerTrajectoryFromChoreo(
              "c_sc_FAR_" + reefs.get(i).toString(), mirrorLengthwise, 1);

      scoringCommand.addCommands(shrimpleCoral(scTraj, intTraj, toGoal(levels.get(i))));
    }

    return Commands.sequence(
        resetPose(traj1.getInitialPose()),
        Commands.waitSeconds(autoSelector.getDelayInput()),
        scoringCommand);
  }

  private Command shrimpleCoral(
      PathPlannerTrajectory scTraj, PathPlannerTrajectory intTraj, WantedSuperState goal) {
    return Commands.none();
    // Commands.sequence(
    //     Commands.sequence(
    //             new DriveTrajectory(drive, scTraj),
    //             Commands.waitUntil(
    //                 () ->
    //                     superstructure.atGoal()
    //                         && superstructure.getGoal() != WantedSuperState.STOW),
    //             scoreCommand(superstructure))
    //         .deadlineFor(
    //             Commands.waitSeconds(scTraj.getTotalTimeSeconds() - 0.75)
    //                 .andThen(superstructure.setGoalCommand(goal))),
    //     superstructure
    //         .setGoalCommand(WantedSuperState.INTAKE_CORAL)
    //         .withDeadline(
    //             new DriveTrajectory(drive, intTraj)
    //                 .andThen(
    //                     Constants.isSim
    //                         ? Commands.waitSeconds(intakeWaitTime)
    //                         : Commands.waitUntil(() -> superstructure.hasCoral()))));
  }

  public Command shrimpleOcrAuto(List<AutoQuestionResponse> reefs) {
    return shrimpleOcrAuto(reefs, reefsToLevels(reefs));
  }

  public Command rpShrimpleOcrAuto() {
    return shrimpleOcrAuto(List.of(AutoQuestionResponse.G))
        .withDeadline(
            Commands.waitUntil( // Cancel rest of path after first score
                () ->
                    !superstructure.hasCoral()
                        && superstructure.getCurrentSuperState()
                            == CurrentSuperState.INTAKE_CORAL));
  }

  public Command shrimpleOcrAuto() {
    List<AutoQuestionResponse> reefs = new ArrayList<>();

    for (int i = 1; i < 5; i++) {
      int ordinal = autoSelector.getResponses().get(i).ordinal() - AutoQuestionResponse.A.ordinal();
      if (ordinal >= 0 && ordinal < 12) {
        reefs.add(autoSelector.getResponses().get(i));
      }
    }

    return shrimpleOcrAuto(reefs);
  }

  private Superstructure.WantedSuperState toGoal(AutoQuestionResponse response) {
    switch (response) {
      case L1_LEFT:
        return Superstructure.WantedSuperState.SCORE_LEFT_L1;

      case L1_RIGHT:
        return Superstructure.WantedSuperState.SCORE_LEFT_L1;

      case L2:
        return Superstructure.WantedSuperState.SCORE_LEFT_L2;

      case L3:
        return Superstructure.WantedSuperState.SCORE_LEFT_L3;

      default:
        return Superstructure.WantedSuperState.STOW;
    }
  }

  public List<AutoQuestionResponse> reefsToLevels(List<AutoQuestionResponse> reefs) {
    if (reefs.isEmpty()) {
      return List.of();
    }

    List<AutoQuestionResponse> levels = new ArrayList<>();

    /** fms convention (A-L) */
    int[] totalAvailable = new int[] {2, 2, 1, 1, 2, 2, 1, 1, 2, 2, 1, 1};

    for (var reef : reefs) {
      int currentReef = reef.ordinal() - AutoQuestionResponse.A.ordinal();
      int available = totalAvailable[currentReef];

      if (available > 1) {
        levels.add(AutoQuestionResponse.L2);
        totalAvailable[currentReef]--;

      } else {
        levels.add(
            available % 2 == 0 ? AutoQuestionResponse.L1_LEFT : AutoQuestionResponse.L1_RIGHT);
        totalAvailable[currentReef]--;
      }
    }

    return levels;
  }
}
