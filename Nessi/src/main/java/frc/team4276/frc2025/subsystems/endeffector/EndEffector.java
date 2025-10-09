package frc.team4276.frc2025.subsystems.endeffector;

import static frc.team4276.frc2025.subsystems.endeffector.EndEffectorConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.frc2025.Constants;
import frc.team4276.frc2025.SimManager;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
  private final EndEffectorIO io;
  private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();

  public enum WantedState {
    IDLE,
    INTAKE,
    SCORE,
    SCORE_RIGHT_L1,
    SCORE_LEFT_L1,
    PURGE
  }

  private enum SystemState {
    IDLE(() -> 0.0),
    INTAKING(intakeVolts),
    INTAKING_SLOW(intakeSlowVolts),
    SCORING(scoreVolts),
    REVERSING(reverseVolts),
    SCORING_RIGHT_L1(favorVolts, lagVolts),
    SCORING_LEFT_L1(lagVolts, favorVolts),
    PURGE(purgeVolts);

    private final DoubleSupplier rightVoltageGoal;
    private final DoubleSupplier leftVoltageGoal;

    private SystemState(DoubleSupplier leftVoltageGoal, DoubleSupplier rightVoltageGoal) {
      this.leftVoltageGoal = leftVoltageGoal;
      this.rightVoltageGoal = rightVoltageGoal;
    }

    private SystemState(DoubleSupplier voltageGoal) {
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

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLE;

  public EndEffector(EndEffectorIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("EndEffector", inputs);

    systemState = handleStateTransition();
    io.runVolts(systemState.getLeftVolts(), systemState.getRightVolts());
    Logger.recordOutput("EndEffector/WantedState", wantedState);
    Logger.recordOutput("EndEffector/SystemState", systemState);
  }

  private SystemState handleStateTransition() {
    return switch (wantedState) {
      case INTAKE:
        {
          if ((inputs.backTripped && inputs.frontRead && systemState == SystemState.REVERSING)
              || (systemState == SystemState.IDLE && hasCoral())) {
            yield SystemState.IDLE;

          } else if ((inputs.backCleared && systemState == SystemState.INTAKING_SLOW)
              || systemState == SystemState.REVERSING) {
            yield SystemState.REVERSING;

          } else if ((inputs.backTripped
                  && !inputs.frontRead
                  && systemState == SystemState.INTAKING)
              || systemState == SystemState.INTAKING_SLOW) {
            yield SystemState.INTAKING_SLOW;

          } else {
            yield SystemState.INTAKING;
          }
        }
      case SCORE:
        {
          yield SystemState.SCORING;
        }
      case SCORE_RIGHT_L1:
        {
          yield SystemState.SCORING_RIGHT_L1;
        }
      case SCORE_LEFT_L1:
        {
          yield SystemState.SCORING_LEFT_L1;
        }
      case PURGE:
        {
          yield SystemState.PURGE;
        }
      default:
        yield SystemState.IDLE;
    };
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  public boolean hasCoral() {
    return Constants.isSim ? SimManager.hasCoral() : inputs.frontRead;
  }
}
