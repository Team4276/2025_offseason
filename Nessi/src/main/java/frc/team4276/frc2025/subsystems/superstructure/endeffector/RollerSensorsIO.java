package frc.team4276.frc2025.subsystems.superstructure.endeffector;

import org.littletonrobotics.junction.AutoLog;

public interface RollerSensorsIO {
  @AutoLog
  abstract class RollerSensorsIOInputs {
    public boolean frontRead = false;
    public boolean frontTripped = false;
    public boolean frontCleared = false;

    public boolean backRead = false;
    public boolean backTripped = false;
    public boolean backCleared = false;
  }

  default void updateInputs(RollerSensorsIOInputs inputs) {}
}
