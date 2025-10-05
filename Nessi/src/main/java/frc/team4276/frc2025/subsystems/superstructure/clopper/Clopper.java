package frc.team4276.frc2025.subsystems.superstructure.clopper;

import static frc.team4276.frc2025.subsystems.superstructure.clopper.ClopperConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.frc2025.SimManager;
import frc.team4276.frc2025.subsystems.superstructure.clopper.climber.ClimberIO;
import frc.team4276.frc2025.subsystems.superstructure.clopper.climber.ClimberIOInputsAutoLogged;
import frc.team4276.frc2025.subsystems.superstructure.clopper.hopper.HopperIO;
import frc.team4276.frc2025.subsystems.superstructure.clopper.hopper.HopperIOInputsAutoLogged;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Clopper extends SubsystemBase {
  public enum WantedState {
    IDLE(0.0, 0.0, 0.0),
    STOW(stowPosition, 0.0, 0.0),
    CLIMB_PREP(climbPosition, climberClimbPrepPosition, wheelClimbVolts),
    CLIMB(climbPosition, climberClimbPosition, 0.0);

    private final double hopperSetpoint;
    private final double climberSetpoint;
    private final double wheelVolts;

    private WantedState(double hopperSetpoint, double climberSetpoint, double wheelVolts) {
      this.hopperSetpoint = hopperSetpoint;
      this.climberSetpoint = climberSetpoint;
      this.wheelVolts = wheelVolts;
    }

    private double getHopperPosition() {
      return hopperSetpoint;
    }

    private double getClimberPosition() {
      return climberSetpoint;
    }

    private double getWheelVolts() {
      return wheelVolts;
    }
  }

  private WantedState wantedState = WantedState.STOW;

  private final ClimberIO climberIo;
  private final ClimberIOInputsAutoLogged climberInputs = new ClimberIOInputsAutoLogged();
  private BooleanSupplier climberOverride = () -> false;
  private boolean climberHasFlippedCoast = false;

  private final HopperIO leftIo;
  private final HopperIO rightIo;
  private BooleanSupplier hopperOverride = () -> false;
  private boolean hopperHasFlippedCoast = false;

  private final HopperIOInputsAutoLogged leftInputs = new HopperIOInputsAutoLogged();
  private final HopperIOInputsAutoLogged rightInputs = new HopperIOInputsAutoLogged();

  private double leftOffset = 0.0;
  private double rightOffset = 0.0;

  public Clopper(ClimberIO climberIo, HopperIO leftIo, HopperIO rightIo) {
    this.climberIo = climberIo;
    this.leftIo = leftIo;
    this.rightIo = rightIo;
  }

  public void setCoastOverride(
      BooleanSupplier hopperCoastOverride, BooleanSupplier climberCoastOverride) {
    this.hopperOverride = hopperCoastOverride;
    this.climberOverride = climberCoastOverride;
  }

  @Override
  public void periodic() {
    climberIo.updateInputs(climberInputs);
    leftIo.updateInputs(leftInputs);
    rightIo.updateInputs(rightInputs);
    Logger.processInputs("Clopper/Climber", climberInputs);
    Logger.processInputs("Clopper/Hopper/Left", leftInputs);
    Logger.processInputs("Clopper/Hopper/Right", rightInputs);

    handleBrakeMode();
    applyState();

    SimManager.addHopperLeftMeasuredObs(leftInputs.position - leftOffset);
    SimManager.addHopperRightMeasuredObs(rightInputs.position - rightOffset);
    SimManager.addHopperGoalObs(wantedState.getHopperPosition());
    SimManager.addClimberMeasuredObs(climberInputs.position);

    Logger.recordOutput("Clopper/WantedState", wantedState.toString());
    Logger.recordOutput("Clopper/Hopper/WantedStatePosition", wantedState.getHopperPosition());
    Logger.recordOutput("Clopper/Hopper/Left/Offset", leftOffset);
    Logger.recordOutput("Clopper/Hopper/Right/Offset", rightOffset);
    Logger.recordOutput("Clopper/Climber/Position", climberInputs.position);
  }

  private void handleBrakeMode() {
    if (!hopperOverride.getAsBoolean()) {
      hopperHasFlippedCoast = true;
    }

    if (!(hopperOverride.getAsBoolean() && hopperHasFlippedCoast)
        && (leftInputs.isCoast && rightInputs.isCoast)) {
      leftOffset = leftInputs.position;
      rightOffset = rightInputs.position;
    }

    leftIo.setBrakeMode(!(hopperOverride.getAsBoolean() && hopperHasFlippedCoast));
    rightIo.setBrakeMode(!(hopperOverride.getAsBoolean() && hopperHasFlippedCoast));

    if (!climberOverride.getAsBoolean()) {
      climberHasFlippedCoast = true;
    }

    climberIo.setBrakeMode(!(climberOverride.getAsBoolean() && climberHasFlippedCoast));
  }

  private void applyState() {
    climberIo.runWheelsAtVolts(wantedState.getWheelVolts());
    climberIo.runMotionMagikSetpoint(wantedState.getClimberPosition());

    leftIo.runMotionMagikSetpoint(wantedState.getHopperPosition() + leftOffset, kS);
    rightIo.runMotionMagikSetpoint(wantedState.getHopperPosition() + rightOffset, kS);
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  public WantedState getWantedState() {
    return wantedState;
  }
}
