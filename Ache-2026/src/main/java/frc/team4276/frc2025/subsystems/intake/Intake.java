package frc.team4276.frc2025.subsystems.intake;

import static frc.team4276.frc2025.subsystems.intake.IntakeConstants.pivotStowPosition;
import static frc.team4276.frc2025.subsystems.intake.IntakeConstants.rollerPassiveEjectVolts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  public enum WantedState {
    IDLE,
    STOW,
    INTAKE,
    STAGE,
    HOLD,
    CLEAR_ARM,
    L1_SCORE,
    DEPLOY,
    EJECT
  }

  private enum SystemState {
    IDLING,
    STOWED,
    INTAKING,
    STAGING,
    HOLDING,
    CLEARING_ARM,
    L1_SCORING,
    DEPLOYING,
    EJECTING
  }

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLING;

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    systemState = handleStateTransition();
    applyState();
  }

  private SystemState handleStateTransition() {
    return switch (wantedState) {
      default: {
        yield SystemState.IDLING;
      }
    };
  }

  private void applyState() {
    switch (systemState) {
      case IDLING:
        io.runPivotVolts(0.0);
        io.runRollerVolts(0.0);

        break;
      case STOWED:
        io.runPivotSetpoint(pivotStowPosition);
        io.runRollerVolts(rollerPassiveEjectVolts);

        break;
      case INTAKING:
        break;
      case STAGING:
        break;
      case HOLDING:
        break;
      case CLEARING_ARM:
        break;
      case L1_SCORING:
        break;
      case DEPLOYING:
        break;
      case EJECTING:
        break;
    }
  }
}
