package frc.team4276.frc2025.subsystems.superstructure.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public boolean leaderMotorConnected = true;
    public boolean followerMotorConnected = true;

    public double position = 0.0;
    public double velocity = 0.0;

    public boolean topLimit = false;
    public boolean botLimit = false;

    public double[] appliedVolts = new double[] {0.0, 0.0};
    public double[] currentAmps = new double[] {0.0, 0.0};
    public double[] tempCelcius = new double[] {0.0, 0.0};
    public boolean absoluteEncoderConnected = true;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  /** Run to setpoint */
  public default void runSetpoint(double setpoint, double ff) {}

  /** Run to setpoint */
  public default void runSetpoint(double setpoint) {}

  /** Run motors at volts */
  public default void runVolts(double volts) {}

  /** Run motors at current */
  public default void runCurrent(double amps) {}

  /** Set brake mode enabled */
  public default void setBrakeMode(boolean enabled) {}

  /** Set PID values */
  public default void setPID(double p, double i, double d) {}

  public default void setPosition(double position) {}

  /** Stops motors */
  public default void stop() {}
}
