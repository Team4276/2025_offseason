package frc.team4276.frc2025.subsystems.superstructure.endeffector;

import edu.wpi.first.wpilibj.DriverStation;
import frc.team4276.frc2025.subsystems.superstructure.RollerSensorsIO;
import frc.team4276.frc2025.subsystems.superstructure.RollerSensorsIOInputsAutoLogged;
import frc.team4276.util.dashboard.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class EndEffector {
  private static final LoggedTunableNumber favorVolts =
      new LoggedTunableNumber("EndEffector/FavorVolts", 4.5);
  private static final LoggedTunableNumber lagVolts =
      new LoggedTunableNumber("EndEffector/LagVolts", 2.0);

  public enum Goal {
    IDLE(() -> 0.0, () -> 0.0),
    INTAKE(new LoggedTunableNumber("EndEffector/IntakeVolts", 5.0)),
    SLOINTAKE(new LoggedTunableNumber("EndEffector/SlowIntakeVolts", 3.0)),
    SCORE(new LoggedTunableNumber("EndEffector/ScoreVolts", 4.0)),
    REVERSE(new LoggedTunableNumber("EndEffector/ReverseVolts", -1.0)),
    FAVOR_LEFT(favorVolts, lagVolts),
    FAVOR_RIGHT(lagVolts, favorVolts);

    private final DoubleSupplier rightVoltageGoal;
    private final DoubleSupplier leftVoltageGoal;

    private Goal(DoubleSupplier leftVoltageGoal, DoubleSupplier rightVoltageGoal) {
      this.leftVoltageGoal = leftVoltageGoal;
      this.rightVoltageGoal = rightVoltageGoal;
    }

    private Goal(DoubleSupplier voltageGoal) {
      this.leftVoltageGoal = voltageGoal;
      this.rightVoltageGoal = voltageGoal;
    }

    public double getLeftVolts() {
      return leftVoltageGoal.getAsDouble();
    }

    public double getRightVolts() {
      return rightVoltageGoal.getAsDouble();
    }
  }

  private Goal goal = Goal.IDLE;
  private Goal currGoal = Goal.IDLE;

  private final EndEffectorIO io;
  private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();

  private final RollerSensorsIO sensorsIO;
  private final RollerSensorsIOInputsAutoLogged sensorsInputs =
      new RollerSensorsIOInputsAutoLogged();

  private boolean hasCoral = false;

  public EndEffector(EndEffectorIO io, RollerSensorsIO sensorsIO) {
    this.io = io;
    this.sensorsIO = sensorsIO;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("EndEffector", inputs);

    sensorsIO.updateInputs(sensorsInputs);
    Logger.processInputs("EndEffector/BeamBreak", sensorsInputs);

    if (DriverStation.isDisabled()) {
      currGoal = Goal.IDLE;
    }

    if (currGoal == Goal.INTAKE && sensorsInputs.backRead) {
      currGoal = Goal.SLOINTAKE;

    } else if (currGoal == Goal.SLOINTAKE && !sensorsInputs.backRead) {
      currGoal = Goal.REVERSE;

    } else if (currGoal == Goal.REVERSE && sensorsInputs.backRead) {
      currGoal = Goal.IDLE;

    } else if ((goal == Goal.INTAKE || goal == Goal.SLOINTAKE)
        && (currGoal != Goal.IDLE || sensorsInputs.frontRead)) {
      // Continue staging process

    } else {
      currGoal = goal;
      hasCoral = sensorsInputs.frontRead;
    }

    io.runVolts(currGoal.getLeftVolts(), currGoal.getRightVolts());
    Logger.recordOutput("EndEffector/Goal", goal);
    Logger.recordOutput("EndEffector/CurrentGoal", currGoal);
  }

  public Goal getGoal() {
    return goal;
  }

  public void setGoal(Goal goal) {
    this.goal = goal;
  }

  public boolean hasCoral() {
    return hasCoral;
  }
}
