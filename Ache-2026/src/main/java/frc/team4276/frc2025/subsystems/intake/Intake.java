package frc.team4276.frc2025.subsystems.intake;

import static frc.team4276.frc2025.subsystems.intake.IntakeConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  public enum WantedState {
    IDLE,
    STOW,
    TUCK,
    INTAKE,
    STAGE,
    STAGE_EJECT,
    CLEAR_ARM,
    L1_SCORE,
    DEPLOY,
    EJECT
  }

  private enum SystemState {
    IDLING,
    STOWED,
    TUCKED,
    INTAKING,
    STAGING,
    STAGE_EJECTING,
    CLEARING_ARM,
    L1_SCORING,
    DEPLOYING,
    EJECTING
  }

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLING;

  private boolean hasCoral = false;

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
      case IDLE:
        yield SystemState.IDLING;
      case STOW:
        yield SystemState.STOWED;
      case TUCK:
        if (hasCoral) {
          yield SystemState.STOWED;
        }
        yield SystemState.TUCKED;
      case INTAKE:
        if (hasCoral) {
          yield SystemState.STOWED;
        }

        yield SystemState.INTAKING;
      case STAGE:
        if (!hasCoral) {
          yield SystemState.STOWED;
        }

        yield SystemState.STAGING;

      case STAGE_EJECT:
        if (!hasCoral) {
          yield SystemState.STOWED;
        }

        yield SystemState.STAGE_EJECTING;
      case CLEAR_ARM:
        yield SystemState.CLEARING_ARM;
      case L1_SCORE:
        yield SystemState.L1_SCORING;
      case DEPLOY:
        yield SystemState.DEPLOYING;
      case EJECT:
        yield SystemState.EJECTING;
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
        if (hasCoral) {
          io.runRollerVolts(rollerHoldVolts);

        } else {
          io.runRollerVolts(rollerPassiveEjectVolts);

        }

        break;
      case TUCKED:
        io.runPivotSetpoint(pivotTuckPosition);
        io.runRollerVolts(rollerPassiveEjectVolts);

        break;
      case INTAKING:
        io.runPivotSetpoint(pivotIntakePosition);
        io.runRollerVolts(rollerIntakeVolts);

        break;
      case STAGING:
        io.runPivotSetpoint(pivotStagePosition);
        io.runRollerVolts(rollerHoldVolts);

        break;
      case STAGE_EJECTING:
        io.runPivotSetpoint(pivotStagePosition);
        io.runRollerVolts(rollerStageEjectVolts);

        break;
      case CLEARING_ARM:
        io.runPivotSetpoint(pivotClearPosition);

        break;
      case L1_SCORING:
        io.runPivotSetpoint(pivotScorePosition);

        break;
      case DEPLOYING:
        io.runPivotSetpoint(pivotDeployPosition);

        break;
      case EJECTING:
        io.runPivotSetpoint(pivotEjectPosition);
        
        break;
    }
  }

  public boolean pivotAtSetpoint(){
    return MathUtil.isNear(0.0, 0.0, 0.0);  
  }
}
