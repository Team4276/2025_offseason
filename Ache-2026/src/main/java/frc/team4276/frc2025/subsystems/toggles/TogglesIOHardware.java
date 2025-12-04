package frc.team4276.frc2025.subsystems.toggles;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import frc.team4276.frc2025.Ports;

public class TogglesIOHardware implements TogglesIO {
  private final DigitalInput elevatorCoastSwitch;
  private final DigitalInput intakeCoastSwitch;

  public TogglesIOHardware() {
    elevatorCoastSwitch = new DigitalInput(Ports.ELEVATOR_COAST_SWITCH);
    intakeCoastSwitch = new DigitalInput(Ports.INTAKE_COAST_SWITCH);
  }

  @Override
  public void updateInputs(TogglesIOInputs inputs) {
    inputs.elevatorCoast = elevatorCoastSwitch.get();
    inputs.intakeCoast = intakeCoastSwitch.get();
    if (DriverStation.isDisabled()) {
      if (!elevatorCoastSwitch.get()) {
        inputs.elevatorCoastFlippedSinceDisable = true;
      }

      if(!intakeCoastSwitch.get()){
        inputs.intakeCoastFlippedSinceDisable = true;
      }

    } else {
      inputs.elevatorCoastFlippedSinceDisable = false;
      inputs.intakeCoastFlippedSinceDisable = false;
    }
  }
}
