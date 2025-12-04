package frc.team4276.frc2025.subsystems.elevator;

import static frc.team4276.frc2025.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.frc2025.Constants;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private ElevatorPosition wantedElevatorPose = ElevatorPosition.STOW;

  public enum WantedState {
    IDLE,
    MOVE_TO_POSITION,
    CUSTOM
  }

  private enum SystemState {
    IDLING,
    MOVING_TO_POSITION,
    CUSTOM
  }

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLING;

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private BooleanSupplier coastOverride = () -> false;

  public Elevator(ElevatorIO io) {
    this.io = io;
    io.setBrakeMode(true);
  }

  public void setCoastOverride(BooleanSupplier coastOverride) {
    this.coastOverride = coastOverride;
  }

  private boolean hasFlippedCoast = false;
  private boolean wasDisabled = true;

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    systemState = handleStateTransitions();
    applyStates();

    if (DriverStation.isDisabled()) {
      wasDisabled = true;

      io.stop();

      if (!coastOverride.getAsBoolean()) {
        hasFlippedCoast = true;
      }

      io.setBrakeMode(!(coastOverride.getAsBoolean() && hasFlippedCoast));

    } else {
      if (wasDisabled) {
        io.setBrakeMode(true);
        wasDisabled = false;
        hasFlippedCoast = false;
      }
    }

    Logger.recordOutput("Elevator/WantedElevatorPose", wantedElevatorPose);
    Logger.recordOutput("Elevator/AtGoal", atGoal());
    Logger.recordOutput("Elevator/PositionMetres", getPositionMetres());
  }

  private SystemState handleStateTransitions() {
    return switch (wantedState) {
      case IDLE:
        yield SystemState.IDLING;
      case MOVE_TO_POSITION:
        yield SystemState.MOVING_TO_POSITION;
      case CUSTOM:
        yield SystemState.CUSTOM;
    };
  }

  private void applyStates() {
    switch (systemState) {
      case IDLING:
        io.runVolts(0.0);

        break;

      case MOVING_TO_POSITION:
        io.runSetpoint(MathUtil.clamp(wantedElevatorPose.getPositionMetres(), minInput, maxInput));

        break;

      case CUSTOM:
        break;
    }
  }

  public void setWantedState(WantedState wantedState, ElevatorPosition elevatorPosition) {
    this.wantedState = wantedState;
    this.wantedElevatorPose = elevatorPosition;
  }

  public boolean atGoal() {
    return Constants.isSim
        ? true
        : MathUtil.isNear(wantedElevatorPose.getPositionMetres(), getPositionMetres(), tolerance);
  }

  public double metresToRotations(double metres) {
    return metres * motorRotationsPerMetre;
  }

  public double rotationsToMetres(double rotations) {
    return rotations / motorRotationsPerMetre;
  }

  public double getPositionMetres() {
    return rotationsToMetres(inputs.position);
  }
}
