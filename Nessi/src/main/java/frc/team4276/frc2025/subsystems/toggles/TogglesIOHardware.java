package frc.team4276.frc2025.subsystems.toggles;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.team4276.frc2025.Ports;

public class TogglesIOHardware implements TogglesIO {
  private final DigitalInput elevatorCoastOverride;
  private final DigitalInput climberCoastOverride;
  private final DigitalInput hopperCoastOverride;

  public TogglesIOHardware() {
    elevatorCoastOverride = new DigitalInput(Ports.ELEVATOR_COAST_OVERRIDE);
    climberCoastOverride = new DigitalInput(Ports.CLIMBER_COAST_OVERRIDE);
    hopperCoastOverride = new DigitalInput(Ports.HOPPER_COAST_OVERRIDE);
  }

  @Override
  public void updateInputs(TogglesIOInputs inputs) {
    inputs.elevatorCoastOverride = elevatorCoastOverride.get();
    inputs.climberCoastOverride = climberCoastOverride.get();
    inputs.hopperCoastOverride = hopperCoastOverride.get();
  }
}
