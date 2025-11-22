package frc.team4276.frc2025.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean[] isConnected = {true, true};
    public boolean isEncoderConnected = true;

    public double position = 0.0;
    public double absoluteEncoderPosition = 0.0;
    public double velocity = 0.0;

    public double[] appliedVolts = {0.0, 0.0};
    public double[] supplyCurrentAmps = {0.0, 0.0};
    public double[] torqueCurrentAmps = {0.0, 0.0};
    public double[] tempCelcius = {0.0, 0.0};
  }

  default void updateInputs(IntakeIOInputs inputs) {
  }

  default void runPivotVolts(double volts) {
  }

  default void runRollerVolts(double volts) {
  }

  default void stop() {
  }

  default void runPivotSetpoint(double pivotPosition) {
  }


  default void setBrakeMode(boolean enabled) {
  }
}
