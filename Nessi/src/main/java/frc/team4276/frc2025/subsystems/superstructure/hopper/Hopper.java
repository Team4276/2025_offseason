package frc.team4276.frc2025.subsystems.superstructure.hopper;

import static frc.team4276.frc2025.subsystems.superstructure.hopper.HopperConstants.kS_left;
import static frc.team4276.frc2025.subsystems.superstructure.hopper.HopperConstants.kS_right;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.frc2025.Constants;
import frc.team4276.frc2025.SimManager;
import frc.team4276.util.dashboard.LoggedTunableNumber;
import frc.team4276.util.dashboard.LoggedTunableProfile;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  public enum WantedState {
    IDLE(() -> 0.0),
    STOW(() -> 0.0),
    CLIMB(new LoggedTunableNumber("Hopper/ClimbPosition", 5.0));

    private final DoubleSupplier setpointSupplier;

    private WantedState(DoubleSupplier setpointSupplier) {
      this.setpointSupplier = setpointSupplier;
    }

    private double getPosition() {
      return setpointSupplier.getAsDouble();
    }
  }

  private WantedState wantedState = WantedState.STOW;
  private double leftOffset = 0.0;
  private double rightOffset = 0.0;

  private final HopperIO leftIo;
  private final HopperIO rightIo;
  private BooleanSupplier override = () -> false;
  private boolean hasFlippedCoast = false;

  private final HopperIOInputsAutoLogged leftInputs = new HopperIOInputsAutoLogged();
  private final HopperIOInputsAutoLogged rightInputs = new HopperIOInputsAutoLogged();
  private ElevatorFeedforward ff_left =
      new ElevatorFeedforward(
          HopperConstants.kS_left.getAsDouble(),
          HopperConstants.kG.getAsDouble(),
          HopperConstants.kV.getAsDouble());
  private ElevatorFeedforward ff_right =
      new ElevatorFeedforward(
          HopperConstants.kS_right.getAsDouble(),
          HopperConstants.kG.getAsDouble(),
          HopperConstants.kV.getAsDouble());
  private final LoggedTunableProfile profile = new LoggedTunableProfile("Hopper", 10.0, 10.0);
  private TrapezoidProfile.State prevLeftState = new TrapezoidProfile.State();
  private TrapezoidProfile.State prevRightState = new TrapezoidProfile.State();

  public Hopper(HopperIO leftIo, HopperIO rightIo) {
    this.leftIo = leftIo;
    this.rightIo = rightIo;
  }

  public void setCoastOverride(BooleanSupplier coastOverride) {
    this.override = coastOverride;
  }

  @Override
  public void periodic() {
    leftIo.updateInputs(leftInputs);
    rightIo.updateInputs(rightInputs);
    Logger.processInputs("Hopper/Left", leftInputs);
    Logger.processInputs("Hopper/Right", rightInputs);

    if (Constants.isTuning) {
      ff_left.setKg(HopperConstants.kG.getAsDouble());
      ff_left.setKv(HopperConstants.kV.getAsDouble());

      ff_right.setKg(HopperConstants.kG.getAsDouble());
      ff_right.setKv(HopperConstants.kV.getAsDouble());
    }

    if (!override.getAsBoolean()) {
      hasFlippedCoast = true;
    }

    if (!(override.getAsBoolean() && hasFlippedCoast)
        && (leftInputs.isCoast && rightInputs.isCoast)) {
      leftOffset = leftInputs.position;
      rightOffset = rightInputs.position;
    }

    leftIo.setBrakeMode(!(override.getAsBoolean() && hasFlippedCoast));
    rightIo.setBrakeMode(!(override.getAsBoolean() && hasFlippedCoast));

    if (DriverStation.isDisabled()) {
      wantedState = WantedState.IDLE;

      prevLeftState = new TrapezoidProfile.State(leftInputs.position - leftOffset, 0.0);
      prevRightState = new TrapezoidProfile.State(rightInputs.position - rightOffset, 0.0);

    } else {
      prevLeftState =
          profile.calculate(
              0.02, prevLeftState, new TrapezoidProfile.State(wantedState.getPosition(), 0.0));
      prevRightState =
          profile.calculate(
              0.02, prevRightState, new TrapezoidProfile.State(wantedState.getPosition(), 0.0));

      leftIo.runSetpoint(
          prevLeftState.position + leftOffset,
          ff_left.calculate(prevLeftState.velocity) + kS_left.getAsDouble());
      rightIo.runSetpoint(
          prevRightState.position + rightOffset,
          ff_right.calculate(prevRightState.velocity) + kS_right.getAsDouble());
    }

    SimManager.addHopperLeftMeasuredObs(leftInputs.position - leftOffset);
    SimManager.addHopperRightMeasuredObs(rightInputs.position - rightOffset);
    SimManager.addHopperGoalObs(wantedState.getPosition());

    Logger.recordOutput("Hopper/WantedState", wantedState.toString());
    Logger.recordOutput("Hopper/WantedState", wantedState.toString());
    Logger.recordOutput("Hopper/Left/Offset", leftOffset);
    Logger.recordOutput("Hopper/Left/Setpoint", prevLeftState.position);
    Logger.recordOutput("Hopper/Right/Offset", rightOffset);
    Logger.recordOutput("Hopper/Right/Setpoint", prevRightState.position);
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  public WantedState getWantedState() {
    return wantedState;
  }
}
