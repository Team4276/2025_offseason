package frc.team4276.frc2025.subsystems.superstructure.endeffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
  @AutoLog
  abstract class EndEffectorIOInputs {
    public boolean leftConnected = true;
    public double leftAppliedVoltage = 0.0;
    public double leftSupplyCurrentAmps = 0.0;
    public double leftTempCelsius = 0.0;

    public boolean rightConnected = true;
    public double rightAppliedVoltage = 0.0;
    public double rightSupplyCurrentAmps = 0.0;
    public double rightTempCelsius = 0.0;

    public double torqueCurrentAmps = 0.0;
  }

  default void updateInputs(EndEffectorIOInputs inputs) {}

  /** Run feeder at volts */
  default void runVolts(double leftVolts, double rightVolts) {}

  /** Stop feeder */
  default void stop() {}
}
