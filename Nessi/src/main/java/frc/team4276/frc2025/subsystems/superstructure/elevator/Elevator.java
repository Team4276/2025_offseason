package frc.team4276.frc2025.subsystems.superstructure.elevator;

import static frc.team4276.frc2025.subsystems.superstructure.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.team4276.frc2025.Constants;
import frc.team4276.frc2025.SimManager;
import frc.team4276.util.dashboard.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Elevator {
  public enum Goal {
    STOW(new LoggedTunableNumber("Elevator/StowPosition", Units.inchesToMeters(0.5))),
    INTAKE(new LoggedTunableNumber("Elevator/IntakePosition", Units.inchesToMeters(0.5))),
    UNJAM(new LoggedTunableNumber("Elevator/UnjamPosition", Units.inchesToMeters(15.0))),
    L1(new LoggedTunableNumber("Elevator/L1Position", Units.inchesToMeters(0.0))),
    L2(new LoggedTunableNumber("Elevator/L2Position", Units.inchesToMeters(6.69))),
    L3(new LoggedTunableNumber("Elevator/L3Position", Units.inchesToMeters(21.76))),
    NET_PREP(new LoggedTunableNumber("Elevator/NetPrep", Units.inchesToMeters(0.0))),
    NET_SCORE(new LoggedTunableNumber("Elevator/NetScore", Units.inchesToMeters(21.26))),
    LO_ALGAE(new LoggedTunableNumber("Elevator/LoAlgae", Units.inchesToMeters(0.0))),
    HI_ALGAE(new LoggedTunableNumber("Elevator/HiAlgae", Units.inchesToMeters(17.32))),
    CHARACTERIZING(() -> 0.0),
    CUSTOM(new LoggedTunableNumber("Elevator/CustomSetpoint", 0.0));

    private final DoubleSupplier elevatorSetpointSupplier;

    private Goal(DoubleSupplier elevatorSetpointSupplier) {
      this.elevatorSetpointSupplier = elevatorSetpointSupplier;
    }

    public double getPositionMetres() {
      return elevatorSetpointSupplier.getAsDouble();
    }
  }

  private Goal goal = Goal.STOW;

  private final LoggedTunableNumber maxVel = new LoggedTunableNumber("Elevator/maxVel", 2.75);
  private final LoggedTunableNumber maxAccel = new LoggedTunableNumber("Elevator/maxAccel", 3.0);

  private final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", 0.20);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", 9.0);
  private final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG", 0.15);
  private final LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA", 0.01);

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private ElevatorFeedforward ff =
      new ElevatorFeedforward(
          kS.getAsDouble(), kG.getAsDouble(), kV.getAsDouble(), kA.getAsDouble());
  private TrapezoidProfile profile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(maxVel.getAsDouble(), maxAccel.getAsDouble()));
  private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();

  private BooleanSupplier coastOverride;

  private double characterizationInput = 0.0;

  /** position that reads zero on the elevator */
  private double homedPosition = 0.0;

  private Timer atGoalTimer = new Timer();
  private final double atGoalTime = 0.2;

  public Elevator(ElevatorIO io) {
    this.io = io;
    io.setBrakeMode(true);

    atGoalTimer.restart();
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
        profile =
            new TrapezoidProfile(
                new TrapezoidProfile.Constraints(maxVel.getAsDouble(), maxAccel.getAsDouble()));
      }

    } else {
      if (wasDisabled) {
        io.setBrakeMode(true);
        wasDisabled = false;
        hasFlippedCoast = false;
      }

      if (!atGoal()) {
        atGoalTimer.reset();
      }

      if (goal == Goal.CHARACTERIZING) {
        io.runVolts(characterizationInput);

      } else {
        setpointState =
            profile.calculate(
                0.02, setpointState, new TrapezoidProfile.State(goal.getPositionMetres(), 0.0));
        double setpointRotations =
            metresToRotations(MathUtil.clamp(setpointState.position, minInput, maxInput))
                + homedPosition;
        io.runSetpoint(setpointRotations, ff.calculate(setpointState.velocity));

        Logger.recordOutput("Elevator/SetpointRotations", setpointRotations);
        Logger.recordOutput("Elevator/SetpointState/PosMetres", setpointState.position);
        Logger.recordOutput("Elevator/SetpointState/VelMetres", setpointState.velocity);
        Logger.recordOutput(
            "Elevator/SetpointState/PosRotations", metresToRotations(setpointState.position));
        Logger.recordOutput(
            "Elevator/SetpointState/VelRotations", metresToRotations(setpointState.velocity));
      }
    }

    SimManager.addElevatorGoalObs(goal.getPositionMetres());
    SimManager.addElevatorMeasuredObs(
        Constants.isSim ? goal.getPositionMetres() : getPositionMetres());
    Logger.recordOutput("Elevator/Goal", goal);
    Logger.recordOutput("Elevator/GoalMetres", goal.getPositionMetres());
    Logger.recordOutput("Elevator/GoalRotations", metresToRotations(goal.getPositionMetres()));
    Logger.recordOutput("Elevator/AtGoal", atGoal());
    Logger.recordOutput("Elevator/HomedPositionRotation", homedPosition);
    Logger.recordOutput("Elevator/PositionMetres", getPositionMetres());
  }

  public void setGoal(Goal goal) {
    this.goal = goal;
  }

  public Goal getGoal() {
    return goal;
  }

  public boolean atGoal() {
    return Constants.isSim
        ? true
        : MathUtil.isNear(goal.getPositionMetres(), getPositionMetres(), tolerance);
  }

  public boolean atGoalDebounce() {
    return atGoalTimer.get() > atGoalTime;
  }

  public void runCharacterization(double output) {
    characterizationInput = output;
    goal = Goal.CHARACTERIZING;
  }

  public double getFFCharacterizationVelocity() {
    return rotationsToMetres(inputs.velocity);
  }

  public void endCharacterizaton() {
    characterizationInput = 0.0;
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
