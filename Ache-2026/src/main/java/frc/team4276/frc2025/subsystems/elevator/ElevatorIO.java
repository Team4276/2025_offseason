package frc.team4276.frc2025.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public boolean[] isConnected = new boolean[]{true, true};

    public double position = 0.0;
    public double velocity = 0.0;

    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double[] tempCelcius = new double[]{0.0, 0.0};
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  /** Run to setpoint */
  public default void runSetpoint(double positionInMetres) {}

  /** Run motors at volts */
  public default void runVolts(double volts) {}

  /** Set brake mode enabled */
  public default void setBrakeMode(boolean enabled) {}

  public default void setPosition(double position) {}

  /** Stops motors */
  public default void stop() {}
}
