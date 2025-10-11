package frc.team4276.frc2025.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.frc2025.RobotState;
import frc.team4276.frc2025.RobotState.VisionMode;
import frc.team4276.frc2025.field.FieldConstants;
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
import frc.team4276.util.AllianceFlipUtil;
import frc.team4276.util.hid.ViXController;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class Superstructure extends SubsystemBase {
  private final Drive drive;

  @SuppressWarnings("unused")
  private final Vision vision;

  private final Elevator elevator;
  private final EndEffector endeffector;
  private final Clopper clopper;

  private final TogglesIO togglesIO;
  private final TogglesIOInputsAutoLogged togglesInputs = new TogglesIOInputsAutoLogged();

  private final ViXController controller = new ViXController(0);

  private SuperstructureConstants.AutomationLevel automationLevel = AutomationLevel.AUTO_RELEASE;

  public enum WantedSuperState {
    STOW,
    STOPPED,
    PURGE_GAMEPIECE,
    INTAKE_CORAL,
    SCORE_MANUAL_L1,
    SCORE_MANUAL_L2,
    SCORE_MANUAL_L3,
    SCORE_LEFT_L1,
    SCORE_LEFT_L2,
    SCORE_LEFT_L3,
    SCORE_RIGHT_L1,
    SCORE_RIGHT_L2,
    SCORE_RIGHT_L3,
    MANUAL_SCORE,
    REEF_ALGAE,
    MANUAL_REEF_ALGAE,
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
    SCORE_MANUAL_L2,
    SCORE_MANUAL_L3,
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
    MANUAL_REEF_ALGAE,
    CLIMB_PREP,
    CLIMB,
    CUSTOM
  }

  private WantedSuperState wantedSuperState = WantedSuperState.STOW;
  private CurrentSuperState currentSuperState = CurrentSuperState.STOW;

  private final double elevatorRaiseThreshold = 2.5;
  private boolean shouldEjectCoral = false;

  private SuperstructureConstants.ReefSelectionMethod reefSelectionMethod =
      ReefSelectionMethod.POSE;
  private boolean isL1Mode = false;

  public enum GamePieceState {
    NO_BANANA,
    CORAL
  }

  private GamePieceState gamePieceState = GamePieceState.NO_BANANA;

  private LoggedNetworkBoolean isManualMode =
      new LoggedNetworkBoolean("Superstructure/isManualMode", false);

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
    drive.setGyroCalibrationSwitch(() -> togglesInputs.gyroCalibrationSwitch);
  }

  @Override
  public void periodic() {
    togglesIO.updateInputs(togglesInputs);
    Logger.processInputs("Toggles", togglesInputs);

    if (gamePieceState == GamePieceState.NO_BANANA && endeffector.hasCoral()) {
      controller.rumbleCommand(RumbleType.kBothRumble, 1.0, 1.0).schedule();
    }

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

      case SCORE_MANUAL_L1:
        currentSuperState = CurrentSuperState.SCORE_MANUAL_L1;

        break;

      case SCORE_MANUAL_L2:
        currentSuperState = CurrentSuperState.SCORE_MANUAL_L2;

        break;

      case SCORE_MANUAL_L3:
        currentSuperState = CurrentSuperState.SCORE_MANUAL_L3;

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
          case SCORE_MANUAL_L1:
            currentSuperState = CurrentSuperState.MANUAL_LEFT_L1;

            break;

          case SCORE_LEFT_TELEOP_L1:
            currentSuperState = CurrentSuperState.MANUAL_LEFT_L1;

            break;

          case SCORE_RIGHT_TELEOP_L1:
            currentSuperState = CurrentSuperState.MANUAL_RIGHT_L1;

            break;

          case SCORE_RIGHT_TELEOP_L2:
          case SCORE_LEFT_TELEOP_L2:
          case SCORE_MANUAL_L2:
            currentSuperState = CurrentSuperState.MANUAL_L2;

            break;

          case SCORE_RIGHT_TELEOP_L3:
          case SCORE_LEFT_TELEOP_L3:
          case SCORE_MANUAL_L3:
            currentSuperState = CurrentSuperState.MANUAL_L3;

            break;

          default:
            break;
        }

        break;

      case REEF_ALGAE:
        currentSuperState = CurrentSuperState.REEF_ALGAE;

        break;

      case MANUAL_REEF_ALGAE:
        currentSuperState = CurrentSuperState.MANUAL_REEF_ALGAE;

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

      case SCORE_MANUAL_L2:
        scoreManualL2();
        break;

      case SCORE_MANUAL_L3:
        scoreManualL3();
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

      case MANUAL_REEF_ALGAE:
        manualReefAlgae();
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
    RobotState.getInstance().setVisionMode(VisionMode.REJECT_ALL);
    RobotState.getInstance().setSideToAccept(ScoringSide.BOTH);
    drive.setWantedState(Drive.WantedState.IDLE);
    elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, ElevatorPosition.STOW);
    endeffector.setWantedState(EndEffector.WantedState.IDLE);
    clopper.setWantedState(Clopper.WantedState.IDLE);
  }

  private void stow() {
    RobotState.getInstance().setVisionMode(VisionMode.POSE_BASED);
    RobotState.getInstance().setSideToAccept(ScoringSide.BOTH);
    if (!DriverStation.isAutonomous()) {
      drive.setWantedState(Drive.WantedState.TELEOP);
    }
    elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, ElevatorPosition.STOW);
    endeffector.setWantedState(EndEffector.WantedState.IDLE);
    clopper.setWantedState(Clopper.WantedState.STOW);
  }

  private void purgeGamePiece() {
    elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, ElevatorPosition.INTAKE);
    endeffector.setWantedState(EndEffector.WantedState.PURGE);
  }

  private void intakeCoral() {
    RobotState.getInstance().setVisionMode(VisionMode.POSE_BASED);
    RobotState.getInstance().setSideToAccept(ScoringSide.BOTH);

    shouldEjectCoral = false;

    if (DriverStation.isTeleop() && !isManualMode.get()) {
      var rotation =
          AllianceFlipUtil.apply(
              AllianceFlipUtil.applyY(RobotState.getInstance().getEstimatedPose().getY())
                      < FieldConstants.fieldWidth / 2
                  ? Rotation2d.fromDegrees(55.0)
                  : Rotation2d.fromDegrees(305.0));

      drive.setHeadingAlignRotation(rotation);
    }

    elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, ElevatorPosition.INTAKE);
    endeffector.setWantedState(EndEffector.WantedState.INTAKE);
  }

  private void scoreManualL1() {
    elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, ElevatorPosition.L1);
  }

  private void scoreManualL2() {
    elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, ElevatorPosition.L2);
  }

  private void scoreManualL3() {
    elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, ElevatorPosition.L3);
  }

  private final Timer coralEjectTimer = new Timer();

  private void scoreTeleopL1(ScoringSide side) {
    if (hasCoral()) {
      driveToScoringPoseAndReturnIfObservationIsPresent(side);
    }

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
        coralEjectTimer.restart();
      }

      if (coralEjectTimer.get() > 0.25 && coralEjectTimer.isRunning()) {
        driveToReturnPose();
        coralEjectTimer.stop();
      }
    }
  }

  private void scoreTeleopL2(ScoringSide side) {
    if (hasCoral()) {
      driveToScoringPoseAndReturnIfObservationIsPresent(side);
    }

    if (shouldRaiseToScoringPosition()) {
      elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, ElevatorPosition.L2);
    }

    shouldEjectCoral = isReadyToEjectTeleopCoral();

    if (shouldEjectCoral) {
      if (automationLevel == AutomationLevel.AUTO_RELEASE) {
        endeffector.setWantedState(EndEffector.WantedState.SCORE);
      }

      if (!hasCoral()) {
        coralEjectTimer.restart();
      }

      if (coralEjectTimer.get() > 0.25 && coralEjectTimer.isRunning()) {
        driveToReturnPose();
        coralEjectTimer.stop();
      }
    }
  }

  private void scoreTeleopL3(ScoringSide side) {
    if (hasCoral()) {
      driveToScoringPoseAndReturnIfObservationIsPresent(side);
    }

    if (shouldRaiseToScoringPosition()) {
      elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, ElevatorPosition.L3);
    }

    shouldEjectCoral = isReadyToEjectTeleopCoral();

    if (shouldEjectCoral) {
      if (automationLevel == AutomationLevel.AUTO_RELEASE) {
        endeffector.setWantedState(EndEffector.WantedState.SCORE);
      }

      if (!hasCoral()) {
        coralEjectTimer.restart();
      }

      if (coralEjectTimer.get() > 0.25 && coralEjectTimer.isRunning()) {
        driveToReturnPose();
        coralEjectTimer.stop();
      }
    }
  }

  private void scoreAutoL1(ScoringSide side) {
    RobotState.getInstance()
        .setVisionMode(
            reefSelectionMethod == ReefSelectionMethod.POSE
                ? VisionMode.POSE_BASED
                : VisionMode.ROTATION_BASED);
    RobotState.getInstance().setSideToAccept(ScoringSide.BOTH);

    if (shouldRaiseToScoringPosition()) {
      elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, ElevatorPosition.L1);
    }

    shouldEjectCoral = isReadyToEjectAutoCoral();

    if (shouldEjectCoral) {
      endeffector.setWantedState(
          side == ScoringSide.LEFT
              ? EndEffector.WantedState.SCORE_LEFT_L1
              : EndEffector.WantedState.SCORE_RIGHT_L1);
    }
  }

  private void scoreAutoL2() {
    RobotState.getInstance()
        .setVisionMode(
            reefSelectionMethod == ReefSelectionMethod.POSE
                ? VisionMode.POSE_BASED
                : VisionMode.ROTATION_BASED);

    if (shouldRaiseToScoringPosition()) {
      elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, ElevatorPosition.L2);
    }

    shouldEjectCoral = isReadyToEjectAutoCoral();

    if (shouldEjectCoral) {
      endeffector.setWantedState(EndEffector.WantedState.SCORE);
    }
  }

  private void scoreAutoL3() {
    RobotState.getInstance()
        .setVisionMode(
            reefSelectionMethod == ReefSelectionMethod.POSE
                ? VisionMode.POSE_BASED
                : VisionMode.ROTATION_BASED);

    if (shouldRaiseToScoringPosition()) {
      elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, ElevatorPosition.L3);
    }

    shouldEjectCoral = isReadyToEjectAutoCoral();

    if (shouldEjectCoral) {
      endeffector.setWantedState(EndEffector.WantedState.SCORE);
    }
  }

  private void manualL1(ScoringSide side) {
    elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, ElevatorPosition.L1);
    endeffector.setWantedState(
        side == ScoringSide.LEFT
            ? EndEffector.WantedState.SCORE_LEFT_L1
            : EndEffector.WantedState.SCORE_RIGHT_L1);
  }

  private void manualL2() {
    elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, ElevatorPosition.L2);
    endeffector.setWantedState(EndEffector.WantedState.SCORE);
  }

  private void manualL3() {
    elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, ElevatorPosition.L3);
    endeffector.setWantedState(EndEffector.WantedState.SCORE);
  }

  private void reefAlgae() {
    if (elevator.getWantedElevatorPose() != ElevatorPosition.HIGH_ALGAE
        && elevator.getWantedElevatorPose() != ElevatorPosition.LOW_ALGAE
        && elevator.getWantedElevatorPose() != ElevatorPosition.ALGAE_CHOP) {
      driveToReturnPose();

      if (drive.isAtAutoAlignPose()) {
        elevator.setWantedState(
            Elevator.WantedState.MOVE_TO_POSITION,
            isHighAlgaePickup() ? ElevatorPosition.HIGH_ALGAE : ElevatorPosition.LOW_ALGAE);
      }

    } else {
      if (elevator.atGoal() && elevator.getWantedElevatorPose() != ElevatorPosition.ALGAE_CHOP) {
        driveToAlgaePickupPose();
      }

      if (drive.isAtPose(getClosestAlgaePickup())) {
        elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, ElevatorPosition.ALGAE_CHOP);
        driveToReturnPose();
      }
    }
  }

  private void manualReefAlgae() {
    elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, ElevatorPosition.HIGH_ALGAE);
  }

  private void climbPrep() {
    elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, ElevatorPosition.CLIMB);
    clopper.setWantedState(Clopper.WantedState.CLIMB_PREP);
  }

  private void climb() {
    elevator.setWantedState(Elevator.WantedState.MOVE_TO_POSITION, ElevatorPosition.CLIMB);
    clopper.setWantedState(Clopper.WantedState.CLIMB);
  }

  private void custom() {}

  private boolean driveToScoringPoseAndReturnIfObservationIsPresent(ScoringSide side) {
    RobotState.getInstance()
        .setVisionMode(
            reefSelectionMethod == ReefSelectionMethod.POSE
                ? VisionMode.POSE_BASED
                : VisionMode.ROTATION_BASED);
    RobotState.getInstance().setSideToAccept(ScoringSide.BOTH);

    drive.setAutoAlignPose(FieldConstants.getCoralScorePose(getReefSide(), side));

    return false;
  }

  private boolean driveToAlgaePickupPose() {
    RobotState.getInstance()
        .setVisionMode(
            reefSelectionMethod == ReefSelectionMethod.POSE
                ? VisionMode.POSE_BASED
                : VisionMode.ROTATION_BASED);

    drive.setAutoAlignPose(getClosestAlgaePickup());

    return false;
  }

  private Pose2d getClosestAlgaePickup() {
    return RobotState.getInstance()
        .getEstimatedPose()
        .nearest(
            List.of(
                getReefSide().getLeftReef().getAlgaePickup(),
                getReefSide().getRightReef().getAlgaePickup()));
  }

  private void driveToReturnPose() {
    RobotState.getInstance()
        .setVisionMode(
            reefSelectionMethod == ReefSelectionMethod.POSE
                ? VisionMode.POSE_BASED
                : VisionMode.ROTATION_BASED);

    var reefSide = getReefSide();

    var reef =
        RobotState.getInstance()
            .getEstimatedPose()
            .nearest(
                List.of(reefSide.getLeftReef().getAlign(), reefSide.getRightReef().getAlign()));

    drive.setAutoAlignPose(reef);
  }

  private ReefSide getReefSide() {
    return FieldConstants.getSideFromTagId(
            reefSelectionMethod == ReefSelectionMethod.POSE
                ? RobotState.getInstance().getTagIdFromClosestPoseSide()
                : RobotState.getInstance().getTagIdFromClosestRotationSide())
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
        FieldConstants.getSideFromTagId(RobotState.getInstance().getTagIdFromClosestPoseSide())
            .get();

    return reefSide == ReefSide.AB || reefSide == ReefSide.EF || reefSide == ReefSide.IJ;
  }

  public CurrentSuperState getCurrentSuperState() {
    return currentSuperState;
  }

  public boolean hasCoral() {
    return gamePieceState == GamePieceState.CORAL;
  }

  public boolean isManualMode() {
    return isManualMode.get();
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
      WantedSuperState l1ModeCondition,
      WantedSuperState manualCondition) {

    return Commands.either(
        Commands.either(
            Commands.either(
                setStateCommand(l1ModeCondition),
                setStateCommand(hasCoralCondition),
                () -> isL1Mode),
            setStateCommand(noPieceCondition),
            this::hasCoral),
        setStateCommand(manualCondition),
        () -> !isManualMode.get());
  }

  // public void toggleReefSelectionMethod() {
  // reefSelectionMethod = (reefSelectionMethod == ReefSelectionMethod.POSE)
  // ? ReefSelectionMethod.ROTATION
  // : ReefSelectionMethod.POSE;
  // }

  public void setL1ModeEnabled(boolean enabled) {
    isL1Mode = enabled;
  }
}
