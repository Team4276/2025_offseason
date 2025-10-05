package frc.team4276.frc2025.subsystems.clopper.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
  @AutoLog
  public static class HopperIOInputs {
    public boolean leaderMotorConnected = true;
    public boolean followerMotorConnected = true;

    public double position = 0.0;
    public double velocity = 0.0;

    public boolean isCoast = false;

    public double[] appliedVolts = new double[] {0.0};
    public double[] currentAmps = new double[] {0.0};
    public double[] tempCelcius = new double[] {0.0};
    public boolean absoluteEncoderConnected = true;
  }

  public default void updateInputs(HopperIOInputs inputs) {}

  /** Run to setpoint angle in radians */
  public default void runSetpoint(double setpointRads, double ff) {}

  /** Run to setpoint angle in radians */
  public default void runSetpoint(double setpointRads) {}

  public default void runMotionMagikSetpoint(double setpointRads, double ff) {}

  /** Run motors at volts */
  public default void runVolts(double volts) {}

  /** Run motors at current */
  public default void runCurrent(double amps) {}

  /** Set brake mode enabled */
  public default void setBrakeMode(boolean enabled) {}

  /** Set PID values */
  public default void setPID(double p, double i, double d) {}

  /** Stops motors */
  public default void stop() {}
}
