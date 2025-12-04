package frc.team4276.frc2025.subsystems.toggles;

import org.littletonrobotics.junction.AutoLog;

public interface TogglesIO {
  @AutoLog
  public static class TogglesIOInputs {
    public boolean elevatorCoast = false;
    public boolean elevatorCoastFlippedSinceDisable = false;
    
    public boolean intakeCoast = false;
    public boolean intakeCoastFlippedSinceDisable = false;
  }

  default void updateInputs(TogglesIOInputs inputs) {
  }
}
