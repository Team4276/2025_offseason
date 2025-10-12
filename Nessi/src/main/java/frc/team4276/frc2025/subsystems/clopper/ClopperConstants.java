package frc.team4276.frc2025.subsystems.clopper;

public class ClopperConstants {
  public static final double climberMaxVel = 100;
  public static final double climberMaxAccel = 200;

  public static final double climberkV = 0.24;
  public static final double climberkP = 0.5;

  public static final double climberStowPosition = 0.0;
  public static final double climberClimbPrepPosition = 200.0;
  public static final double climberClimbPosition = 0.0;

  public static final double wheelClimbVolts = 12.0;

  public static final boolean invertRight = true;
  public static final boolean invertleft = false;

  public static final int currentLimit = 40;
  public static final int readFreq = 50;

  public static final double encoderPositionFactor = 1.0;
  public static final double encoderVelocityFactor = 1.0 / 60.0;
  public static final double radsPerMotorRotation = 2 * Math.PI / 20;
  public static final boolean invertEncoder = false;

  public static final double kp = 0.5;
  public static final double ki = 0.0;
  public static final double kd = 0.0;

  public static final double kS = 0.5;

  public static final double stowPosition = 0.0;
  public static final double climbPosition = 5.0;
}
