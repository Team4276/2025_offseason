package frc.team4276.frc2025.subsystems.superstructure;

import frc.team4276.frc2025.Ports;
import frc.team4276.util.drivers.BeamBreak;

public class RollerSensorsIOHardware implements RollerSensorsIO {
  private final BeamBreak frontBeamBreak;
  private final BeamBreak backBeamBreak;

  public RollerSensorsIOHardware() {
    frontBeamBreak = new BeamBreak(Ports.CORAL_BREAK_FRONT);
    backBeamBreak = new BeamBreak(Ports.CORAL_BREAK_BACK);
  }

  @Override
  public void updateInputs(RollerSensorsIOInputs inputs) {
    frontBeamBreak.update();
    backBeamBreak.update();

    inputs.frontRead = frontBeamBreak.get();
    inputs.frontTripped = frontBeamBreak.wasTripped();
    inputs.frontCleared = frontBeamBreak.wasCleared();

    inputs.backRead = backBeamBreak.get();
    inputs.backTripped = backBeamBreak.wasTripped();
    inputs.backCleared = backBeamBreak.wasCleared();
  }
}
