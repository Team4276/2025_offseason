package frc.team4276.frc2025.subsystems.elevator;

import static frc.team4276.frc2025.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.frc2025.Constants;
import frc.team4276.frc2025.SimManager;
import frc.team4276.util.dashboard.LoggedTunableProfile;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private ElevatorPosition wantedElevatorPose = ElevatorPosition.STOW;

  private enum WantedState {
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

  private ElevatorFeedforward ff =
      new ElevatorFeedforward(
          kS.getAsDouble(), kG.getAsDouble(), kV.getAsDouble(), kA.getAsDouble());
  private final LoggedTunableProfile profile = new LoggedTunableProfile("Elevator", 1.5, 20.0);
  private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();

  private BooleanSupplier coastOverride = () -> false;

  /** position that reads zero on the elevator */
  private double homedPosition = 0.0;

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

    if (inputs.botLimit) {
      homedPosition = inputs.position;
    }

    systemState = handleStateTransitions();
    applyStates();

    if (DriverStation.isDisabled()) {
      wasDisabled = true;

      io.stop();

      if (!coastOverride.getAsBoolean()) {
        hasFlippedCoast = true;
      }

      io.setBrakeMode(!(coastOverride.getAsBoolean() && hasFlippedCoast));

      setpointState = new TrapezoidProfile.State(getPositionMetres(), 0.0);

      if (Constants.isTuning) {
        ff =
            new ElevatorFeedforward(
                kS.getAsDouble(), kG.getAsDouble(), kV.getAsDouble(), kA.getAsDouble());
      }

    } else {
      if (wasDisabled) {
        io.setBrakeMode(true);
        wasDisabled = false;
        hasFlippedCoast = false;
      }

      Logger.recordOutput("Elevator/SetpointState/PosMetres", setpointState.position);
      Logger.recordOutput("Elevator/SetpointState/VelMetres", setpointState.velocity);
    }

    SimManager.addElevatorMeasuredObs(getPositionMetres());
    Logger.recordOutput("Elevator/Goal", wantedElevatorPose);
    Logger.recordOutput("Elevator/AtGoal", atGoal());
    Logger.recordOutput("Elevator/HomedPositionRotation", homedPosition);
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
        setpointState =
            profile.calculate(
                0.02,
                setpointState,
                new TrapezoidProfile.State(wantedElevatorPose.getPositionMetres(), 0.0));
        double setpointRotations =
            metresToRotations(MathUtil.clamp(setpointState.position, minInput, maxInput))
                + homedPosition;
        io.runSetpoint(setpointRotations, ff.calculate(setpointState.velocity));

        break;

      case CUSTOM:
        break;
    }
  }

  public void setWantedState(WantedState wantedState, ElevatorPosition elevatorPosition) {
    this.wantedState = wantedState;
    this.wantedElevatorPose = elevatorPosition;
  }

  public ElevatorPosition getWantedElevatorPose() {
    return wantedElevatorPose;
  }

  public boolean atGoal() {
    return MathUtil.isNear(wantedElevatorPose.getPositionMetres(), getPositionMetres(), tolerance);
  }

  public static double metresToRotations(double metres) {
    return (metres / drumCircumference) * gearRatio;
  }

  public static double rotationsToMetres(double rotations) {
    return (rotations / gearRatio) * drumCircumference;
  }

  public double getPositionMetres() {
    return rotationsToMetres(inputs.position) - rotationsToMetres(homedPosition);
  }
}
