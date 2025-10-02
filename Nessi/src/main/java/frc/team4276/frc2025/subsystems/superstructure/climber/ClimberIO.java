package frc.team4276.frc2025.subsystems.superstructure.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double appliedVoltageWhench = 0.0;
    public boolean whenchConnected = true;
    public double whenchSupplyCurrentAmps = 0.0;
    public double whenchTempCelsius = 0.0;

    public double appliedVoltageWheels = 0.0;
    public boolean wheelsConnected = true;
    public double wheelsSupplyCurrentAmps = 0.0;
    public double wheelsTempCelsius = 0.0;

    public double position = 0.0;
  }

  default void updateInputs(ClimberIOInputs inputs) {}

  default void runWheelsAtVolts(double volts) {}

  default void runRunWhenchAtVolts(double volts) {}

  public default void setBrakeMode(boolean enabled) {}
}
