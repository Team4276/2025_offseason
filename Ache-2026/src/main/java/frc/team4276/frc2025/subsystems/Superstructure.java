package frc.team4276.frc2025.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.frc2025.RobotState;
import frc.team4276.frc2025.field.FieldConstants;
import frc.team4276.frc2025.field.FieldConstants.ScoringSide;
import frc.team4276.frc2025.subsystems.drive.Drive;
import frc.team4276.frc2025.subsystems.elevator.Elevator;
import frc.team4276.frc2025.subsystems.elevator.ElevatorConstants.ElevatorPosition;
import frc.team4276.frc2025.subsystems.intake.Intake;
import frc.team4276.frc2025.subsystems.toggles.TogglesIO;
import frc.team4276.frc2025.subsystems.toggles.TogglesIOInputsAutoLogged;
import frc.team4276.frc2025.subsystems.vision.Vision;
import frc.team4276.lib.hid.ViXController;

public class Superstructure extends SubsystemBase {
  private final Drive drive;

  @SuppressWarnings("unused")
  private final Vision vision;

  private final Elevator elevator;
  private final Intake intake;
  private final ViXController controller;

  private final TogglesIO togglesIO;
  private final TogglesIOInputsAutoLogged togglesInputs = new TogglesIOInputsAutoLogged();

  public enum WantedSuperState {
    STOW,
    REEF_TEST_LEFT,
    REEF_TEST_RIGHT,
    INTAKE_CORAL,
    SCORE_L1_INTAKE_LEFT,
    SCORE_L1_INTAKE_RIGHT,
    PURGE
  }

  public enum CurrentSuperState {
    STOWED,
    REEF_TESTING_LEFT,
    REEF_TESTING_RIGHT,
    INTAKING_CORAL,
    SCORING_L1_INTAKE_LEFT,
    SCORING_L1_INTAKE_RIGHT,
    PURGING
  }

  private WantedSuperState wantedSuperState = WantedSuperState.STOW;
  private CurrentSuperState currentSuperState = CurrentSuperState.STOWED;

  public enum GamePieceState {
    NO_BANANA,
    CORAL,
    ALGAE
  }

  private GamePieceState gamePieceState = GamePieceState.NO_BANANA;

  private boolean isL1Mode = true;

  public Superstructure(ViXController controller, Drive drive, Vision vision, Elevator elevator, Intake intake,
      TogglesIO togglesIO) {
    this.controller = controller;
    this.drive = drive;
    this.vision = vision;
    this.elevator = elevator;
    this.intake = intake;
    this.togglesIO = togglesIO;
  }

  @Override
  public void periodic() {
    togglesIO.updateInputs(togglesInputs);
    Logger.processInputs("Toggles", togglesInputs);

    if (intake.hasCoral()) {
      if (gamePieceState == GamePieceState.NO_BANANA) {
        CommandScheduler.getInstance().schedule(controller.rumbleCommand(RumbleType.kBothRumble, 1.0, 1.0));
      }

      gamePieceState = GamePieceState.CORAL;
    } else {
      gamePieceState = GamePieceState.NO_BANANA;

    }

    currentSuperState = handleStateTransition();
    applyState();

    Logger.recordOutput("Superstructure/WantedSuperState", wantedSuperState);
    Logger.recordOutput("Superstructure/CurrentSuperState", currentSuperState);
  }

  private CurrentSuperState handleStateTransition() {
    return switch (wantedSuperState) {
      case STOW:
        yield CurrentSuperState.STOWED;
      case REEF_TEST_LEFT:
        yield CurrentSuperState.REEF_TESTING_LEFT;
      case REEF_TEST_RIGHT:
        yield CurrentSuperState.REEF_TESTING_RIGHT;
      case INTAKE_CORAL:
        yield CurrentSuperState.INTAKING_CORAL;
      case SCORE_L1_INTAKE_LEFT:
        yield CurrentSuperState.SCORING_L1_INTAKE_LEFT;
      case SCORE_L1_INTAKE_RIGHT:
        yield CurrentSuperState.SCORING_L1_INTAKE_RIGHT;
      case PURGE:
        yield CurrentSuperState.PURGING;
    };
  }

  private void applyState() {
    switch (currentSuperState) {
      case STOWED:
        stow();

        break;
      case REEF_TESTING_LEFT:
        reefTesting(ScoringSide.LEFT);

        break;
      case REEF_TESTING_RIGHT:
        reefTesting(ScoringSide.RIGHT);

        break;

      case INTAKING_CORAL:
        intakeCoral();

        break;
      case SCORING_L1_INTAKE_LEFT:
        scoreL1Intake(ScoringSide.LEFT);

        break;
      case SCORING_L1_INTAKE_RIGHT:
        scoreL1Intake(ScoringSide.LEFT);

        break;
      case PURGING:
        purge();

        break;
    }
  }

  private void stow() {
    intake.setWantedState(Intake.WantedState.STOW);
    drive.setWantedState(Drive.WantedState.TELEOP);
    elevator.setWantedState(Elevator.WantedState.IDLE, ElevatorPosition.STOW);
  }

  private void reefTesting(ScoringSide side) {
    intake.setWantedState(Intake.WantedState.STOW);
    FieldConstants.getCoralScorePose(RobotState.getInstance().getTagIdFromClosestPoseSide(), side)
        .ifPresent(
            (pose) -> drive.setAutoAlignPose(pose));
    elevator.setWantedState(Elevator.WantedState.IDLE, ElevatorPosition.STOW);

  }

  private void intakeCoral() {
    intake.setWantedState(Intake.WantedState.INTAKE);
    drive.setWantedState(Drive.WantedState.TELEOP);
    elevator.setWantedState(Elevator.WantedState.IDLE, ElevatorPosition.STOW);

  }

  private void scoreL1Intake(ScoringSide side) {
    intake.setWantedState(Intake.WantedState.L1_SCORE);
    drive.setWantedState(Drive.WantedState.TELEOP);
    // FieldConstants.getCoralScorePose(RobotState.getInstance().getTagIdFromClosestPoseSide(),
    // side)
    // .ifPresent(
    // (pose) -> drive.setAutoAlignPose(pose));
    elevator.setWantedState(Elevator.WantedState.IDLE, ElevatorPosition.STOW);

  }

  private void purge() {
    intake.setWantedState(Intake.WantedState.PURGE);
    drive.setWantedState(Drive.WantedState.TELEOP);
    elevator.setWantedState(Elevator.WantedState.IDLE, ElevatorPosition.STOW);

  }

  public boolean hasCoral() {
    return gamePieceState == GamePieceState.CORAL;
  }

  public boolean hasL1Coral() {
    return intake.hasCoral();
  }

  public boolean hasAlgae() {
    return gamePieceState == GamePieceState.ALGAE;
  }

  public void setWantedSuperState(WantedSuperState state) {
    wantedSuperState = state;
  }

  public Command setStateCommand(WantedSuperState superState) {
    return Commands.runOnce(() -> setWantedSuperState(superState));
  }

  public Command configureButtonBinding(
      WantedSuperState hasCoralCondition,
      WantedSuperState algaeCondition,
      WantedSuperState l1ModeCondition,
      WantedSuperState noPieceCondition) {

    return Commands.either(
        setStateCommand(algaeCondition),
        Commands.either(
            Commands.either(
                setStateCommand(l1ModeCondition),
                setStateCommand(hasCoralCondition),
                () -> isL1Mode),
            setStateCommand(noPieceCondition),
            this::hasCoral),
        this::hasAlgae);
  }

  public void setL1ModeEnabled(boolean enabled) {
    isL1Mode = enabled;
  }
}
