package frc.team4276.frc2025.subsystems.superstructure;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.frc2025.subsystems.superstructure.SuperstructureConstants.AutomationLevel;
import frc.team4276.frc2025.subsystems.superstructure.SuperstructureConstants.ReefSelectionMethod;
import frc.team4276.frc2025.subsystems.superstructure.climber.Climber;
import frc.team4276.frc2025.subsystems.superstructure.drive.Drive;
import frc.team4276.frc2025.subsystems.superstructure.elevator.Elevator;
import frc.team4276.frc2025.subsystems.superstructure.endeffector.EndEffector;
import frc.team4276.frc2025.subsystems.superstructure.hopper.Hopper;
import frc.team4276.frc2025.subsystems.vision.Vision;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  private final Drive drive;
  private final Vision vision;
  private final Elevator elevator;
  private final EndEffector endeffector;
  private final Hopper hopper;
  private final Climber climber;

  private SuperstructureConstants.AutomationLevel automationLevel = AutomationLevel.AUTO_RELEASE;

  public enum WantedSuperState {
    STOW,
    STOPPED,
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

  public enum GamePieceState {
    NO_BANANA,
    CORAL,
    L1_MODE
  }

  private GamePieceState gamePieceState = GamePieceState.NO_BANANA;

  public Superstructure(
      Drive drive,
      Vision vision,
      Elevator elevator,
      EndEffector endeffector,
      Hopper hopper,
      Climber climber) {
    this.drive = drive;
    this.vision = vision;
    this.elevator = elevator;
    this.endeffector = endeffector;
    this.hopper = hopper;
    this.climber = climber;
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
        break;

      case INTAKE_CORAL:
        break;

      case SCORE_MANUAL_L1:
        break;

      case SCORE_LEFT_TELEOP_L1:
        break;

      case SCORE_LEFT_TELEOP_L2:
        break;

      case SCORE_LEFT_TELEOP_L3:
        break;

      case SCORE_RIGHT_TELEOP_L1:
        break;

      case SCORE_RIGHT_TELEOP_L2:
        break;

      case SCORE_RIGHT_TELEOP_L3:
        break;

      case SCORE_LEFT_AUTO_L1:
        break;

      case SCORE_RIGHT_AUTO_L1:
        break;

      case SCORE_LEFT_AUTO_L2:
        break;

      case SCORE_LEFT_AUTO_L3:
        break;

      case SCORE_RIGHT_AUTO_L2:
        break;

      case SCORE_RIGHT_AUTO_L3:
        break;

      case MANUAL_LEFT_L1:
        break;

      case MANUAL_RIGHT_L1:
        break;

      case MANUAL_L2:
        break;

      case MANUAL_L3:
        break;

      case REEF_ALGAE:
        break;

      case CLIMB_PREP:
        break;

      case CLIMB:
        break;

      case CUSTOM:
        break;

      case STOPPED:
        break;

      default:
        break;
    }
  }

  public CurrentSuperState getCurrentSuperState() {
    return currentSuperState;
  }

  public boolean hasCoral() {
    return gamePieceState == GamePieceState.CORAL;
  }

  public void setCoastOverride(
      BooleanSupplier elevatorOverride,
      BooleanSupplier climberOverride,
      BooleanSupplier hopperOverride) {
    elevator.setCoastOverride(elevatorOverride);
    climber.setCoastOverride(climberOverride);
    hopper.setCoastOverride(hopperOverride);
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
          yield setStateCommand(hasCoralCondition);
        }
      case NO_BANANA:
        {
          yield setStateCommand(noPieceCondition);
        }
      case L1_MODE:
        {
          yield setStateCommand(l1ModeCondition);
        }
    };
  }
}
