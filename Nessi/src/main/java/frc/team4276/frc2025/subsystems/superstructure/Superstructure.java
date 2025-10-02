package frc.team4276.frc2025.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.frc2025.Constants;
import frc.team4276.frc2025.SimManager;
import frc.team4276.frc2025.subsystems.superstructure.elevator.Elevator;
import frc.team4276.frc2025.subsystems.superstructure.endeffector.EndEffector;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  private final Elevator elevator;
  private final EndEffector endeffector;

  private boolean wantScore = false;
  private boolean leftL1 = false;

  public enum Goal {
    STOW,
    INTAKE,
    L1,
    L2,
    L3,
    LO_ALGAE,
    HI_ALGAE,
    SHUFFLE,
    CLIMB,
    CHARACTERIZING,
    CUSTOM
  }

  private Goal desiredGoal = Goal.STOW;
  private Goal currentGoal = Goal.STOW;
  private Goal autoScoreGoal = Goal.STOW;

  private double elevatorCharacterizationInput = 0.0;

  public Superstructure(Elevator elevator, EndEffector endeffector) {
    this.elevator = elevator;
    this.endeffector = endeffector;

    elevator.setCoastOverride(() -> false);

    setDefaultCommand(setGoalCommand(() -> Superstructure.Goal.STOW));
  }

  @Override
  public void periodic() {
    if (desiredGoal == Goal.STOW
        && currentGoal == Goal.INTAKE
        && (endeffector.getGoal() == EndEffector.Goal.SLOINTAKE
            || endeffector.getGoal() == EndEffector.Goal.REVERSE)) {
      // Continue intaking

    } else {
      currentGoal = desiredGoal;
    }

    if (wantScore) {
      endeffector.setGoal(
          currentGoal == Goal.L1
              ? (leftL1 ? EndEffector.Goal.FAVOR_LEFT : EndEffector.Goal.FAVOR_RIGHT)
              : EndEffector.Goal.SCORE);
    } else {
      endeffector.setGoal(EndEffector.Goal.IDLE);
    }

    switch (currentGoal) {
      case STOW:
        elevator.setGoal(Elevator.Goal.STOW);
        endeffector.setGoal(EndEffector.Goal.IDLE);

        break;

      case SHUFFLE:
        endeffector.setGoal(EndEffector.Goal.SLOINTAKE);

        break;

      case INTAKE:
        elevator.setGoal(Elevator.Goal.INTAKE);
        endeffector.setGoal(EndEffector.Goal.INTAKE);

        break;

      case L1:
        elevator.setGoal(Elevator.Goal.L1);

        break;

      case L2:
        elevator.setGoal(Elevator.Goal.L2);

        break;

      case L3:
        elevator.setGoal(Elevator.Goal.L3);

        break;

      case LO_ALGAE:
        elevator.setGoal(Elevator.Goal.LO_ALGAE);
        endeffector.setGoal(EndEffector.Goal.IDLE);

        break;
      case HI_ALGAE:
        elevator.setGoal(Elevator.Goal.HI_ALGAE);
        endeffector.setGoal(EndEffector.Goal.IDLE);

        break;

      case CLIMB:
        elevator.setGoal(Elevator.Goal.STOW);
        endeffector.setGoal(EndEffector.Goal.IDLE);

        break;
      case CHARACTERIZING:
        elevator.runCharacterization(elevatorCharacterizationInput);
        endeffector.setGoal(endEffectorGoal);
        break;

      case CUSTOM:
        elevator.setGoal(Elevator.Goal.CUSTOM);

      default:
        break;
    }

    elevator.periodic();
    endeffector.periodic();

    Logger.recordOutput("Superstructure/DesiredGoal", desiredGoal);
    Logger.recordOutput("Superstructure/CurrentGoal", currentGoal);
    Logger.recordOutput("Superstructure/WantScore", wantScore);
    Logger.recordOutput("Superstructure/HasCoral", endeffector.hasCoral());
  }

  public boolean atGoal() {
    return elevator.atGoal();
  }

  public boolean withinTolerance(double tol) {
    return Math.abs(elevator.getGoal().getPositionMetres() - elevator.getPositionMetres()) < tol;
  }

  public Goal getGoal() {
    return currentGoal;
  }

  public void selectAutoScoreGoal(Goal goal) {
    autoScoreGoal = goal;
  }

  public Command autoScoreCommand() {
    return startEnd(
        () -> {
          desiredGoal = autoScoreGoal;
        },
        () -> {
          desiredGoal = Goal.STOW;
        });
  }

  public void setGoal(Supplier<Goal> goal) {
    desiredGoal = goal.get();
  }

  public Command setGoalCommand(Supplier<Goal> goal) {
    return startEnd(() -> setGoal(goal), () -> setGoal(() -> Goal.STOW));
  }

  public Command setGoalCommand(Goal goal) {
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
    setGoal(() -> Goal.CHARACTERIZING);
  }

  public double getFFCharacterizationVelocity() {
    return elevator.getFFCharacterizationVelocity();
  }

  public void endCharacterizaton() {
    elevator.endCharacterizaton();
  }

  private EndEffector.Goal endEffectorGoal = EndEffector.Goal.IDLE;

  public void setEndEffectorGoal(EndEffector.Goal goal) {
    endEffectorGoal = goal;
  }

  public void setCoastOverride(BooleanSupplier override) {
    elevator.setCoastOverride(override);
  }

  public boolean hasCoral() {
    return Constants.isSim ? SimManager.hasCoral() : endeffector.hasCoral();
  }
}
