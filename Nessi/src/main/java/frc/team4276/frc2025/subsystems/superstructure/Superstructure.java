package frc.team4276.frc2025.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.frc2025.Constants;
import frc.team4276.frc2025.SimManager;
import frc.team4276.frc2025.subsystems.superstructure.elevator.Elevator;
import frc.team4276.frc2025.subsystems.superstructure.elevator.ElevatorConstants;
import frc.team4276.frc2025.subsystems.superstructure.endeffector.EndEffector;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  private final Elevator elevator;
  private final EndEffector endeffector;

  private boolean wantScore = false;
  private boolean leftL1 = false;

  public enum WantedSuperState {
    STOW,
    STOPPED,
    INTAKE,
    L1,
    L2,
    L3,
    LO_ALGAE,
    HI_ALGAE,
    CLIMB,
    CHARACTERIZING,
    CUSTOM
  }

  public enum CurrentSuperState {
    STOW,
    STOPPED,
    INTAKE,
    L1,
    L2,
    L3,
    LO_ALGAE,
    HI_ALGAE,
    CLIMB,
    CHARACTERIZING,
    CUSTOM
  }

  private WantedSuperState wantedSuperState = WantedSuperState.STOW;
  private CurrentSuperState currentSuperState = CurrentSuperState.STOW;

  private double elevatorCharacterizationInput = 0.0;

  public Superstructure(Elevator elevator, EndEffector endeffector) {
    this.elevator = elevator;
    this.endeffector = endeffector;

    elevator.setCoastOverride(() -> false);

    setDefaultCommand(setGoalCommand(() -> Superstructure.WantedSuperState.STOW));
  }

  @Override
  public void periodic() {
    currentSuperState = handleStateTransition();
    applyState();

    elevator.periodic();
    endeffector.periodic();

    Logger.recordOutput("Superstructure/WantedSuperState", wantedSuperState);
    Logger.recordOutput("Superstructure/CurrentSuperState", currentSuperState);
    Logger.recordOutput("Superstructure/WantScore", wantScore);
    Logger.recordOutput("Superstructure/HasCoral", endeffector.hasCoral());
  }

  private CurrentSuperState handleStateTransition() {
    return switch (wantedSuperState) {
      default:
        yield CurrentSuperState.STOPPED;

      case STOW:
        yield CurrentSuperState.STOW;

      case L1:
        yield CurrentSuperState.L1;

      case L2:
        yield CurrentSuperState.L2;

      case L3:
        yield CurrentSuperState.L3;

      case LO_ALGAE:
        yield CurrentSuperState.LO_ALGAE;

      case HI_ALGAE:
        yield CurrentSuperState.HI_ALGAE;

      case CLIMB:
        yield CurrentSuperState.CLIMB;

      case CHARACTERIZING:
        yield CurrentSuperState.CHARACTERIZING;

      case CUSTOM:
        yield CurrentSuperState.CUSTOM;
    };
  }

  private void applyState() {
    if (wantScore) {
      endeffector.setWantedState(
          wantedSuperState == WantedSuperState.L1
              ? (leftL1
                  ? EndEffector.WantedState.SCORE_LEFT_L1
                  : EndEffector.WantedState.SCORE_RIGHT_L1)
              : EndEffector.WantedState.SCORE);
    } else {
      endeffector.setWantedState(EndEffector.WantedState.IDLE);
    }

    switch (currentSuperState) {
      case STOW:
        elevator.setGoal(ElevatorConstants.Goal.STOW);
        endeffector.setWantedState(EndEffector.WantedState.IDLE);

        break;

      case INTAKE:
        elevator.setGoal(ElevatorConstants.Goal.INTAKE);
        endeffector.setWantedState(EndEffector.WantedState.INTAKE);

        break;

      case L1:
        elevator.setGoal(ElevatorConstants.Goal.L1);

        break;

      case L2:
        elevator.setGoal(ElevatorConstants.Goal.L2);

        break;

      case L3:
        elevator.setGoal(ElevatorConstants.Goal.L3);

        break;

      case LO_ALGAE:
        elevator.setGoal(ElevatorConstants.Goal.LO_ALGAE);
        endeffector.setWantedState(EndEffector.WantedState.IDLE);

        break;
      case HI_ALGAE:
        elevator.setGoal(ElevatorConstants.Goal.HI_ALGAE);
        endeffector.setWantedState(EndEffector.WantedState.IDLE);

        break;

      case CLIMB:
        elevator.setGoal(ElevatorConstants.Goal.STOW);
        endeffector.setWantedState(EndEffector.WantedState.IDLE);

        break;
      case CHARACTERIZING:
        elevator.runCharacterization(elevatorCharacterizationInput);
        break;

      case CUSTOM:
        elevator.setGoal(ElevatorConstants.Goal.CUSTOM);

      default:
        break;
    }
  }

  public boolean atGoal() {
    return elevator.atGoal();
  }

  public boolean withinTolerance(double tol) {
    return Math.abs(elevator.getGoal().getPositionMetres() - elevator.getPositionMetres()) < tol;
  }

  public WantedSuperState getGoal() {
    return wantedSuperState;
  }

  public void selectAutoScoreGoal(WantedSuperState goal) {}

  public Command autoScoreCommand() {
    return startEnd(
        () -> {},
        () -> {
          wantedSuperState = WantedSuperState.STOW;
        });
  }

  public void setGoal(Supplier<WantedSuperState> goal) {
    wantedSuperState = goal.get();
  }

  public Command setGoalCommand(Supplier<WantedSuperState> goal) {
    return startEnd(() -> setGoal(goal), () -> setGoal(() -> WantedSuperState.STOW));
  }

  public Command setGoalCommand(WantedSuperState goal) {
    return setGoalCommand(() -> goal);
  }

  public Command scoreCommand(BooleanSupplier isLeftL1) {
    return scoreCommand(isLeftL1.getAsBoolean());
  }

  public Command scoreCommand(boolean isLeftL1) {
    return Commands.startEnd(
        () -> {
          wantScore = true;
          leftL1 = isLeftL1;
        },
        () -> {
          wantScore = false;
          SimManager.setHasCoral(false);
        });
  }

  public void acceptCharacterizationInput(double input) {
    elevatorCharacterizationInput = input;
    setGoal(() -> WantedSuperState.CHARACTERIZING);
  }

  public double getFFCharacterizationVelocity() {
    return elevator.getFFCharacterizationVelocity();
  }

  public void endCharacterizaton() {
    elevator.endCharacterizaton();
  }

  public void setCoastOverride(BooleanSupplier override) {
    elevator.setCoastOverride(override);
  }

  public boolean hasCoral() {
    return Constants.isSim ? SimManager.hasCoral() : endeffector.hasCoral();
  }
}
