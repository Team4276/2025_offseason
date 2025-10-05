package frc.team4276.frc2025.subsystems.superstructure;

import static frc.team4276.frc2025.subsystems.superstructure.SuperstructureConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.frc2025.subsystems.superstructure.clopper.Clopper;
import frc.team4276.frc2025.subsystems.superstructure.drive.Drive;
import frc.team4276.frc2025.subsystems.superstructure.elevator.Elevator;
import frc.team4276.frc2025.subsystems.superstructure.endeffector.EndEffector;
import frc.team4276.frc2025.subsystems.vision.Vision;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  private final Drive drive;
  private final Vision vision;
  private final Elevator elevator;
  private final EndEffector endeffector;
  private final Clopper clopper;

  private SuperstructureConstants.AutomationLevel automationLevel = AutomationLevel.AUTO_RELEASE;

  public enum WantedSuperState {
    STOW,
    STOPPED,
    PURGE_GAMEPIECE,
    INTAKE_CORAL,
    SCORE_MANUAL_L1,
    SCORE_LEFT_L1,
    SCORE_LEFT_L2,
    SCORE_LEFT_L3,
    SCORE_RIGHT_L1,
    SCORE_RIGHT_L2,
    SCORE_RIGHT_L3,
    MANUAL_SCORE,
    REEF_ALGAE,
    CLIMB_PREP,
    CLIMB,
    CUSTOM
  }

  public enum CurrentSuperState {
    STOW,
    STOPPED,
    PURGE_GAMEPIECE,
    INTAKE_CORAL,
    SCORE_MANUAL_L1,
    SCORE_LEFT_TELEOP_L1,
    SCORE_LEFT_TELEOP_L2,
    SCORE_LEFT_TELEOP_L3,
    SCORE_RIGHT_TELEOP_L1,
    SCORE_RIGHT_TELEOP_L2,
    SCORE_RIGHT_TELEOP_L3,
    SCORE_LEFT_AUTO_L1,
    SCORE_RIGHT_AUTO_L1,
    SCORE_LEFT_AUTO_L2,
    SCORE_LEFT_AUTO_L3,
    SCORE_RIGHT_AUTO_L2,
    SCORE_RIGHT_AUTO_L3,
    MANUAL_LEFT_L1,
    MANUAL_RIGHT_L1,
    MANUAL_L2,
    MANUAL_L3,
    REEF_ALGAE,
    CLIMB_PREP,
    CLIMB,
    CUSTOM
  }

  private WantedSuperState wantedSuperState = WantedSuperState.STOPPED;
  private CurrentSuperState currentSuperState = CurrentSuperState.STOPPED;

  private SuperstructureConstants.ReefSelectionMethod reefSelectionMethod =
      ReefSelectionMethod.ROTATION;
  private boolean isL1Mode = false;

  public enum GamePieceState {
    NO_BANANA,
    CORAL
  }

  private GamePieceState gamePieceState = GamePieceState.NO_BANANA;

  public Superstructure(
      Drive drive, Vision vision, Elevator elevator, EndEffector endeffector, Clopper clopper) {
    this.drive = drive;
    this.vision = vision;
    this.elevator = elevator;
    this.endeffector = endeffector;
    this.clopper = clopper;
  }

  @Override
  public void periodic() {
    gamePieceState = endeffector.hasCoral() ? GamePieceState.CORAL : GamePieceState.NO_BANANA;

    currentSuperState = handleStateTransition();
    applyState();

    Logger.recordOutput("Superstructure/WantedSuperState", wantedSuperState);
    Logger.recordOutput("Superstructure/CurrentSuperState", currentSuperState);
    Logger.recordOutput("Superstructure/HasCoral", gamePieceState == GamePieceState.CORAL);
  }

  private CurrentSuperState handleStateTransition() {
    switch (wantedSuperState) {
      default:
        currentSuperState = CurrentSuperState.STOPPED;

        break;

      case STOW:
        currentSuperState = CurrentSuperState.STOW;

        break;

      case PURGE_GAMEPIECE:
        currentSuperState = CurrentSuperState.PURGE_GAMEPIECE;

        break;

      case INTAKE_CORAL:
        currentSuperState = CurrentSuperState.INTAKE_CORAL;

        break;

      case SCORE_LEFT_L1:
        currentSuperState =
            DriverStation.isAutonomous()
                ? CurrentSuperState.SCORE_LEFT_AUTO_L1
                : CurrentSuperState.SCORE_LEFT_TELEOP_L1;

        break;

      case SCORE_LEFT_L2:
        currentSuperState =
            DriverStation.isAutonomous()
                ? CurrentSuperState.SCORE_LEFT_AUTO_L2
                : CurrentSuperState.SCORE_LEFT_TELEOP_L2;

        break;

      case SCORE_LEFT_L3:
        currentSuperState =
            DriverStation.isAutonomous()
                ? CurrentSuperState.SCORE_LEFT_AUTO_L3
                : CurrentSuperState.SCORE_LEFT_TELEOP_L3;

        break;

      case SCORE_RIGHT_L1:
        currentSuperState =
            DriverStation.isAutonomous()
                ? CurrentSuperState.SCORE_RIGHT_AUTO_L1
                : CurrentSuperState.SCORE_RIGHT_TELEOP_L1;

        break;

      case SCORE_RIGHT_L2:
        currentSuperState =
            DriverStation.isAutonomous()
                ? CurrentSuperState.SCORE_RIGHT_AUTO_L2
                : CurrentSuperState.SCORE_RIGHT_TELEOP_L2;

        break;

      case SCORE_RIGHT_L3:
        currentSuperState =
            DriverStation.isAutonomous()
                ? CurrentSuperState.SCORE_RIGHT_AUTO_L3
                : CurrentSuperState.SCORE_RIGHT_TELEOP_L3;

        break;

      case MANUAL_SCORE:
        switch (currentSuperState) {
          case SCORE_LEFT_TELEOP_L1:
            currentSuperState = CurrentSuperState.MANUAL_LEFT_L1;

            break;

          case SCORE_RIGHT_TELEOP_L1:
            currentSuperState = CurrentSuperState.MANUAL_RIGHT_L1;

            break;

          case SCORE_RIGHT_TELEOP_L2:
          case SCORE_LEFT_TELEOP_L2:
            currentSuperState = CurrentSuperState.MANUAL_L2;

            break;

          case SCORE_RIGHT_TELEOP_L3:
          case SCORE_LEFT_TELEOP_L3:
            currentSuperState = CurrentSuperState.MANUAL_L3;

            break;

          default:
            break;
        }

      case REEF_ALGAE:
        currentSuperState = CurrentSuperState.REEF_ALGAE;

        break;

      case CLIMB_PREP:
        currentSuperState = CurrentSuperState.CLIMB_PREP;

        break;

      case CLIMB:
        currentSuperState = CurrentSuperState.CLIMB;

        break;

      case CUSTOM:
        currentSuperState = CurrentSuperState.CUSTOM;

        break;
    }
    ;

    return currentSuperState;
  }

  private void applyState() {
    switch (currentSuperState) {
      case STOW:
        stow();
        break;

      case PURGE_GAMEPIECE:
        purgeGamePiece();
        break;

      case INTAKE_CORAL:
        intakeCoral();
        break;

      case SCORE_MANUAL_L1:
        scoreManualL1();
        break;

      case SCORE_LEFT_TELEOP_L1:
        scoreTeleopL1(ScoringSide.LEFT);
        break;

      case SCORE_LEFT_TELEOP_L2:
        scoreTeleopL2(ScoringSide.LEFT);
        break;

      case SCORE_LEFT_TELEOP_L3:
        scoreTeleopL3(ScoringSide.LEFT);
        break;

      case SCORE_RIGHT_TELEOP_L1:
        scoreTeleopL1(ScoringSide.RIGHT);
        break;

      case SCORE_RIGHT_TELEOP_L2:
        scoreTeleopL2(ScoringSide.RIGHT);
        break;

      case SCORE_RIGHT_TELEOP_L3:
        scoreTeleopL3(ScoringSide.RIGHT);
        break;

      case SCORE_LEFT_AUTO_L1:
        scoreAutoL1(ScoringSide.LEFT);
        break;

      case SCORE_RIGHT_AUTO_L1:
        scoreAutoL1(ScoringSide.LEFT);
        break;

      case SCORE_LEFT_AUTO_L2:
        scoreAutoL2();
        break;

      case SCORE_LEFT_AUTO_L3:
        scoreAutoL3();
        break;

      case SCORE_RIGHT_AUTO_L2:
        scoreAutoL2();
        break;

      case SCORE_RIGHT_AUTO_L3:
        scoreAutoL3();
        break;

      case MANUAL_LEFT_L1:
        manualL1(ScoringSide.LEFT);
        break;

      case MANUAL_RIGHT_L1:
        manualL1(ScoringSide.RIGHT);
        break;

      case MANUAL_L2:
        manualL2();
        break;

      case MANUAL_L3:
        manualL3();
        break;

      case REEF_ALGAE:
        reefAlgae();
        break;

      case CLIMB_PREP:
        climbPrep();
        break;

      case CLIMB:
        climb();
        break;

      case CUSTOM:
        custom();
        break;

      case STOPPED:
        stopped();
        break;

      default:
        break;
    }
  }

  private void stopped() {
    clopper.setWantedState(Clopper.WantedState.IDLE);
  }

  private void stow() {
    clopper.setWantedState(Clopper.WantedState.STOW);
  }

  private void purgeGamePiece() {}

  private void intakeCoral() {}

  private void scoreManualL1() {}

  private void scoreTeleopL1(ScoringSide side) {}

  private void scoreTeleopL2(ScoringSide side) {}

  private void scoreTeleopL3(ScoringSide side) {}

  private void scoreAutoL1(ScoringSide side) {}

  private void scoreAutoL2() {}

  private void scoreAutoL3() {}

  private void manualL1(ScoringSide side) {}

  private void manualL2() {}

  private void manualL3() {}

  private void reefAlgae() {}

  private void climbPrep() {
    clopper.setWantedState(Clopper.WantedState.CLIMB_PREP);
  }

  private void climb() {
    clopper.setWantedState(Clopper.WantedState.CLIMB);
  }

  private void custom() {}

  public CurrentSuperState getCurrentSuperState() {
    return currentSuperState;
  }

  public boolean hasCoral() {
    return gamePieceState == GamePieceState.CORAL;
  }

  public boolean isL1Mode() {
    return isL1Mode;
  }

  public void setCoastOverride(
      BooleanSupplier elevatorOverride,
      BooleanSupplier climberOverride,
      BooleanSupplier hopperOverride) {
    elevator.setCoastOverride(elevatorOverride);
    clopper.setCoastOverride(hopperOverride, climberOverride);
  }

  public void setWantedSuperState(WantedSuperState superState) {
    wantedSuperState = superState;
  }

  public Command setStateCommand(WantedSuperState superState, boolean runDuringClimb) {
    Command command = new InstantCommand(() -> setWantedSuperState(superState));
    if (!runDuringClimb) {
      command =
          command.onlyIf(
              () ->
                  currentSuperState != CurrentSuperState.CLIMB
                      && currentSuperState != CurrentSuperState.CLIMB_PREP);
    }

    return command;
  }

  public Command setStateCommand(WantedSuperState superState) {
    return setStateCommand(superState, false);
  }

  public Command configureButtonBinding(
      WantedSuperState hasCoralCondition,
      WantedSuperState noPieceCondition,
      WantedSuperState l1ModeCondition) {

    return switch (gamePieceState) {
      case CORAL:
        {
          if (isL1Mode) {
            yield setStateCommand(l1ModeCondition);
          }

          yield setStateCommand(hasCoralCondition);
        }
      case NO_BANANA:
        {
          yield setStateCommand(noPieceCondition);
        }
    };
  }

  public void setL1ModeEnabled(boolean enabled) {
    isL1Mode = enabled;
  }
}
