package frc.team4276.frc2025.subsystems.elevator;

import edu.wpi.first.math.util.Units;
import frc.team4276.lib.dashboard.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public class ElevatorConstants {
  public static final LoggedTunableNumber stow =
      new LoggedTunableNumber("Elevator/StowPosition", Units.inchesToMeters(0.5));

  public enum ElevatorPosition {
    STOW(stow);

    private final DoubleSupplier elevatorSetpointSupplier;

    private ElevatorPosition(DoubleSupplier elevatorSetpointSupplier) {
      this.elevatorSetpointSupplier = elevatorSetpointSupplier;
    }

    public double getPositionMetres() {
      return elevatorSetpointSupplier.getAsDouble();
    }
  }

  public static final double kP = 5.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  public static final double kS = 0.2;

  public static final double tolerance = 0.01; // m

  public static final double motorRotationsPerMetre = (64.0 / 12.0) / (30.0 * 0.005);

  public static final double maxVelMps = 1.5; // 2.5 Max
  public static final double maxVelRps = maxVelMps * motorRotationsPerMetre;
  public static final double maxAccelMpss = 10.0; // 16.0 Max
  public static final double maxAccelRpss = maxAccelMpss * motorRotationsPerMetre;

  public static final double minInput = 0.0; // m
  public static final double maxInput = Units.inchesToMeters(49.0); // m

}
