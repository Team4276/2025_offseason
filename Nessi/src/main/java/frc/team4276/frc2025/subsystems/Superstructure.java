package frc.team4276.frc2025.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.frc2025.RobotState;
import frc.team4276.frc2025.RobotState.VisionMode;
import frc.team4276.frc2025.field.FieldConstants.ReefSide;
import frc.team4276.frc2025.subsystems.SuperstructureConstants.AutomationLevel;
import frc.team4276.frc2025.subsystems.SuperstructureConstants.ReefSelectionMethod;
import frc.team4276.frc2025.subsystems.SuperstructureConstants.ScoringSide;
import frc.team4276.frc2025.subsystems.clopper.Clopper;
import frc.team4276.frc2025.subsystems.drive.Drive;
import frc.team4276.frc2025.subsystems.elevator.Elevator;
import frc.team4276.frc2025.subsystems.elevator.ElevatorConstants.ElevatorPosition;
import frc.team4276.frc2025.subsystems.endeffector.EndEffector;
import frc.team4276.frc2025.subsystems.toggles.TogglesIO;
import frc.team4276.frc2025.subsystems.toggles.TogglesIOInputsAutoLogged;
import frc.team4276.frc2025.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  private final Drive drive;
  private final Vision vision;
  private final Elevator elevator;
  private final EndEffector endeffector;
  private final Clopper clopper;

  private final TogglesIO togglesIO;
  private final TogglesIOInputsAutoLogged togglesInputs = new TogglesIOInputsAutoLogged();

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

  private WantedSuperState wantedSuperState = WantedSuperState.STOW;
  private CurrentSuperState currentSuperState = CurrentSuperState.STOW;

  private final double elevatorRaiseThreshold = 2.5;
  private boolean shouldEjectCoral = false;

  private SuperstructureConstants.ReefSelectionMethod reefSelectionMethod =
      ReefSelectionMethod.ROTATION;
  private boolean isL1Mode = false;

  public enum GamePieceState {
    NO_BANANA,
    CORAL
  }

  private GamePieceState gamePieceState = GamePieceState.NO_BANANA;

  public Superstructure(
      Drive drive,
      Vision vision,
      Elevator elevator,
      EndEffector endeffector,
      Clopper clopper,
      TogglesIO togglesIO) {
    this.drive = drive;
    this.vision = vision;
    this.elevator = elevator;
    this.endeffector = endeffector;
    this.clopper = clopper;
    this.togglesIO = togglesIO;

    elevator.setCoastOverride(() -> togglesInputs.elevatorCoastOverride);
    clopper.setCoastOverride(
        () -> togglesInputs.hopperCoastOverride, () -> togglesInputs.climberCoastOverride);
  }

  @Override
  public void periodic() {
    togglesIO.updateInputs(togglesInputs);
    Logger.processInputs("Toggles", togglesInputs);

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
    drive.setWantedState(Drive.WantedState.IDLE);
    elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, ElevatorPosition.STOW);
    endeffector.setWantedState(EndEffector.WantedState.IDLE);
    clopper.setWantedState(Clopper.WantedState.IDLE);
  }

  private void stow() {
    drive.setWantedState(Drive.WantedState.TELEOP);
    elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, ElevatorPosition.STOW);
    endeffector.setWantedState(EndEffector.WantedState.IDLE);
    clopper.setWantedState(Clopper.WantedState.STOW);
  }

  private void purgeGamePiece() {
    drive.setWantedState(Drive.WantedState.TELEOP);
    elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, ElevatorPosition.INTAKE);
    endeffector.setWantedState(EndEffector.WantedState.PURGE);
  }

  private void intakeCoral() {
    shouldEjectCoral = false;
    drive.setWantedState(Drive.WantedState.TELEOP);
    elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, ElevatorPosition.INTAKE);
    endeffector.setWantedState(EndEffector.WantedState.INTAKE);
  }

  private void scoreManualL1() {
    elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, ElevatorPosition.L1);

    if (!hasCoral()) {
      setWantedSuperState(WantedSuperState.STOW);
    }
  }

  private void scoreTeleopL1(ScoringSide side) {
    driveToScoringPoseAndReturnIfObservationPresent(side);

    if (shouldRaiseToScoringPosition()) {
      elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, ElevatorPosition.L1);
    }

    shouldEjectCoral = isReadyToEjectTeleopCoral();

    if (shouldEjectCoral) {
      if (automationLevel == AutomationLevel.AUTO_RELEASE) {
        endeffector.setWantedState(
            side == ScoringSide.LEFT
                ? EndEffector.WantedState.SCORE_LEFT_L1
                : EndEffector.WantedState.SCORE_RIGHT_L1);
      }

      if (!hasCoral()) {
        setWantedSuperState(WantedSuperState.STOW);
        driveToReturnPose(side);
      }
    }
  }

  private void scoreTeleopL2(ScoringSide side) {
    driveToScoringPoseAndReturnIfObservationPresent(side);

    if (shouldRaiseToScoringPosition()) {
      elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, ElevatorPosition.L2);
    }

    shouldEjectCoral = isReadyToEjectTeleopCoral();

    if (shouldEjectCoral) {
      if (automationLevel == AutomationLevel.AUTO_RELEASE) {
        endeffector.setWantedState(EndEffector.WantedState.SCORE);
      }

      if (!hasCoral()) {
        setWantedSuperState(WantedSuperState.STOW);
        driveToReturnPose(side);
      }
    }
  }

  private void scoreTeleopL3(ScoringSide side) {
    driveToScoringPoseAndReturnIfObservationPresent(side);

    if (shouldRaiseToScoringPosition()) {
      elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, ElevatorPosition.L3);
    }

    shouldEjectCoral = isReadyToEjectTeleopCoral();

    if (shouldEjectCoral) {
      if (automationLevel == AutomationLevel.AUTO_RELEASE) {
        endeffector.setWantedState(EndEffector.WantedState.SCORE);
      }

      if (!hasCoral()) {
        setWantedSuperState(WantedSuperState.STOW);
        driveToReturnPose(side);
      }
    }
  }

  private void scoreAutoL1(ScoringSide side) {
    if (shouldRaiseToScoringPosition()) {
      elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, ElevatorPosition.L1);
    }

    shouldEjectCoral = isReadyToEjectAutoCoral();

    if (shouldEjectCoral) {
      if (automationLevel == AutomationLevel.AUTO_RELEASE) {
        endeffector.setWantedState(
            side == ScoringSide.LEFT
                ? EndEffector.WantedState.SCORE_LEFT_L1
                : EndEffector.WantedState.SCORE_RIGHT_L1);
      }
    }
  }

  private void scoreAutoL2() {
    if (shouldRaiseToScoringPosition()) {
      elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, ElevatorPosition.L2);
    }

    shouldEjectCoral = isReadyToEjectAutoCoral();

    if (shouldEjectCoral) {
      if (automationLevel == AutomationLevel.AUTO_RELEASE) {
        endeffector.setWantedState(EndEffector.WantedState.SCORE);
      }
    }
  }

  private void scoreAutoL3() {
    if (shouldRaiseToScoringPosition()) {
      elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, ElevatorPosition.L3);
    }

    shouldEjectCoral = isReadyToEjectAutoCoral();

    if (shouldEjectCoral) {
      if (automationLevel == AutomationLevel.AUTO_RELEASE) {
        endeffector.setWantedState(EndEffector.WantedState.SCORE);
      }
    }
  }

  private void manualL1(ScoringSide side) {
    endeffector.setWantedState(
        side == ScoringSide.LEFT
            ? EndEffector.WantedState.SCORE_LEFT_L1
            : EndEffector.WantedState.SCORE_RIGHT_L1);
  }

  private void manualL2() {
    endeffector.setWantedState(EndEffector.WantedState.SCORE);
  }

  private void manualL3() {
    endeffector.setWantedState(EndEffector.WantedState.SCORE);
  }

  private void reefAlgae() {
    if (elevator.getWantedElevatorPose() != ElevatorPosition.HIGH_ALGAE
        && elevator.getWantedElevatorPose() != ElevatorPosition.LOW_ALGAE) {
      driveToReturnPose(ScoringSide.RIGHT); // TODO: make it work with left and right

      if (drive.isAtAutoAlignPose()) {
        elevator.setWantedState(
            Elevator.WantedState.MOVE_TO_POSITION,
            isHighAlgaePickup() ? ElevatorPosition.HIGH_ALGAE : ElevatorPosition.LOW_ALGAE);
      }

    } else {
      if (elevator.atGoal()) {
        driveToScoringPoseAndReturnIfObservationPresent(ScoringSide.RIGHT);
      }

      if (drive.isAtPose(getReefSide().getSecondReef().getScore())) {
        elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, ElevatorPosition.STOW);
        driveToReturnPose(ScoringSide.RIGHT);
      }
    }
  }

  private void climbPrep() {
    drive.setWantedState(Drive.WantedState.TELEOP);
    elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, ElevatorPosition.CLIMB);
    clopper.setWantedState(Clopper.WantedState.CLIMB_PREP);
  }

  private void climb() {
    drive.setWantedState(Drive.WantedState.TELEOP);
    elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, ElevatorPosition.CLIMB);
    clopper.setWantedState(Clopper.WantedState.CLIMB);
  }

  private void custom() {}

  private boolean driveToScoringPoseAndReturnIfObservationPresent(ScoringSide side) {
    RobotState.getInstance().setVisionMode(VisionMode.ACCEPT_ALL);

    var reefSide = getReefSide();

    var reef = side == ScoringSide.LEFT ? reefSide.getFirstReef() : reefSide.getSecondReef();

    drive.setAutoAlignPose(reef.getScore());

    return false;
  }

  private void driveToReturnPose(ScoringSide side) {
    RobotState.getInstance().setVisionMode(VisionMode.ACCEPT_ALL);

    var reefSide = getReefSide();

    var reef = side == ScoringSide.LEFT ? reefSide.getFirstReef() : reefSide.getSecondReef();

    drive.setAutoAlignPose(reef.getAlign());
  }

  private ReefSide getReefSide() {
    return RobotState.getInstance()
        .getSideFromTagId(RobotState.getInstance().getTagIdFromClosestPoseSide())
        .get();
  }

  private boolean shouldRaiseToScoringPosition() {
    return drive.isAtAutoAlignPose(elevatorRaiseThreshold);
  }

  private boolean isReadyToEjectTeleopCoral() {
    return elevator.atGoal() && drive.isAtAutoAlignPose();
  }

  private boolean isReadyToEjectAutoCoral() {
    return elevator.atGoal() && drive.isAtAutoAlignPose();
  }

  private boolean isHighAlgaePickup() {
    var reefSide =
        RobotState.getInstance()
            .getSideFromTagId(RobotState.getInstance().getTagIdFromClosestPoseSide())
            .get();

    return reefSide == ReefSide.AB || reefSide == ReefSide.EF || reefSide == ReefSide.IJ;
  }

  public CurrentSuperState getCurrentSuperState() {
    return currentSuperState;
  }

  public boolean hasCoral() {
    return gamePieceState == GamePieceState.CORAL;
  }

  public boolean isL1Mode() {
    return isL1Mode;
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

    return Commands.either(
        Commands.either(
            setStateCommand(l1ModeCondition), setStateCommand(hasCoralCondition), () -> isL1Mode),
        setStateCommand(noPieceCondition),
        this::hasCoral);
  }

  public void setL1ModeEnabled(boolean enabled) {
    isL1Mode = enabled;
  }
}
