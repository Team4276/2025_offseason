package frc.team4276.frc2025.subsystems.toggles;

import org.littletonrobotics.junction.AutoLog;

public interface TogglesIO {
  @AutoLog
  public static class TogglesIOInputs {
    public boolean elevatorCoastOverride = false;
    public boolean gyroCalibrationSwitch = false;
    public boolean climberCoastOverride = false;
    public boolean hopperCoastOverride = false;
  }

  default void updateInputs(TogglesIOInputs inputs) {}
}
