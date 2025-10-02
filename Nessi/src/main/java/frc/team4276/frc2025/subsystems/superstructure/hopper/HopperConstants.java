package frc.team4276.frc2025.subsystems.superstructure.hopper;

import frc.team4276.util.dashboard.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public class HopperConstants {
  public static final boolean invertRight = true;
  public static final boolean invertleft = false;

  public static final int currentLimit = 40;

  public static final double encoderPositionFactor = 1.0;
  public static final double encoderVelocityFactor = 1.0 / 60.0;
  public static final boolean invertEncoder = false;

  public static final double kp = 0.01;
  public static final double ki = 0.0;
  public static final double kd = 0.0;

  public static final DoubleSupplier kS_left = new LoggedTunableNumber("Hopper/kS_left", 0.5);
  public static final DoubleSupplier kS_right = new LoggedTunableNumber("Hopper/kS_right", 0.5);

  public static final DoubleSupplier kV = new LoggedTunableNumber("Hopper/kV", 0.0020808);
  public static final DoubleSupplier kG = new LoggedTunableNumber("Hopper/kG", 0);
  public static final DoubleSupplier kA = new LoggedTunableNumber("Hopper/kA", 0);

  public static final int readFreq = 50;

  public static final double minInput = Math.toRadians(90.0);
  public static final double maxInput = Math.toRadians(91.0);

  public static final double radsPerMotorRotation = 2 * Math.PI / 20;
}
