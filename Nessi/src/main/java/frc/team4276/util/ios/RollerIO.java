package frc.team4276.util.ios;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
  @AutoLog
  abstract class RollerIOInputs {
    public boolean connected = true;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  default void updateInputs(RollerIOInputs inputs) {}

  /** Run feeder at volts */
  default void runVolts(double volts) {}

  /** Stop feeder */
  default void stop() {}
}
