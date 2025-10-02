package frc.team4276.frc2025.subsystems.superstructure.displacer;

import frc.team4276.util.dashboard.LoggedTunableNumber;
import frc.team4276.util.ios.RollerIO;
import frc.team4276.util.ios.RollerIOInputsAutoLogged;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Displacer {
  public enum Goal {
    IDLE(() -> 0.0),
    MOOORV(new LoggedTunableNumber("Displacer/MOOORVVolts", 2.0)),
    VROOOM(new LoggedTunableNumber("Displacer/VROOMVolts", -12.0));

    private final DoubleSupplier voltageGoal;

    private Goal(DoubleSupplier voltageGoal) {
      this.voltageGoal = voltageGoal;
    }

    private double getVolts() {
      return voltageGoal.getAsDouble();
    }
  }

  private Goal goal = Goal.IDLE;

  private final RollerIO io;
  private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

  private boolean hasSpiked = false;

  public Displacer(RollerIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Displacer", inputs);

    if (goal != Goal.VROOOM) {
      hasSpiked = false;

    } else if (inputs.supplyCurrentAmps > 20.0) {
      // hasSpiked = true;
    }

    io.runVolts(hasSpiked ? 0.0 : goal.getVolts());

    Logger.recordOutput("Displacer/Goal", goal);
    Logger.recordOutput("Displacer/GoalVolts", goal);
    Logger.recordOutput("Displacer/HasSpiked", hasSpiked);
  }

  @AutoLogOutput
  public Goal getGoal() {
    return goal;
  }

  @AutoLogOutput
  public void setGoal(Goal goal) {
    this.goal = goal;
  }
}
