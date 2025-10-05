package frc.team4276.frc2025.subsystems.elevator;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.team4276.frc2025.Ports;
import frc.team4276.util.dashboard.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public class ElevatorConstants {
  public static final LoggedTunableNumber stow =
      new LoggedTunableNumber("Elevator/StowPosition", Units.inchesToMeters(0.5));
  public static final LoggedTunableNumber intake =
      new LoggedTunableNumber("Elevator/IntakePosition", Units.inchesToMeters(0.5));
  public static final LoggedTunableNumber unjam =
      new LoggedTunableNumber("Elevator/UnjamPosition", Units.inchesToMeters(15.0));
  public static final LoggedTunableNumber l1 =
      new LoggedTunableNumber("Elevator/L1Position", Units.inchesToMeters(0.0));
  public static final LoggedTunableNumber l2 =
      new LoggedTunableNumber("Elevator/L2Position", Units.inchesToMeters(6.69));
  public static final LoggedTunableNumber l3 =
      new LoggedTunableNumber("Elevator/L3Position", Units.inchesToMeters(21.76));
  public static final LoggedTunableNumber lowAlgae =
      new LoggedTunableNumber("Elevator/LoAlgae", Units.inchesToMeters(0.0));
  public static final LoggedTunableNumber highAlgae =
      new LoggedTunableNumber("Elevator/HiAlgae", Units.inchesToMeters(17.32));

  public enum ElevatorPosition {
    STOW(stow),
    INTAKE(intake),
    UNJAM(unjam),
    L1(l1),
    L2(l2),
    L3(l3),
    LOW_ALGAE(lowAlgae),
    HIGH_ALGAE(highAlgae);

    private final DoubleSupplier elevatorSetpointSupplier;

    private ElevatorPosition(DoubleSupplier elevatorSetpointSupplier) {
      this.elevatorSetpointSupplier = elevatorSetpointSupplier;
    }

    public double getPositionMetres() {
      return elevatorSetpointSupplier.getAsDouble();
    }
  }

  public static final int leaderId = Ports.ELEVATOR_LEADER;
  public static final int followerId = Ports.ELEVATOR_FOLLOWER;

  public static final boolean invertLeader = true;
  public static final boolean invertFollower = true;

  public static final int currentLimit = 50;

  public static final double drumDiameter = 0.04375; // m
  public static final double drumCircumference = drumDiameter * Math.PI; // m
  public static final double encoderPositionFactor = 1.0;
  public static final double encoderVelocityFactor = 1.0 / 60.0;
  public static final boolean invertEncoder = true;

  public static final double kp = 0.1;
  public static final double ki = 0.0;
  public static final double kd = 0.0;

  public static final int readFreq = 50;

  public static final double minInput = 0.0; // m
  public static final double maxInput = Units.inchesToMeters(25.5); // m

  public static final double gearRatio = 9.0;
  public static final Translation2d origin =
      new Translation2d(0.0, -Units.inchesToMeters(0.860191));
  public static final double length = Units.inchesToMeters(35);

  public static final double maxPosition = Units.inchesToMeters(25.5); // m

  public static final double tolerance = 0.05; // m

  public static final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", 0.20);
  public static final LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", 7.8);
  public static final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG", 0.15);
  public static final LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA", 0.01);
}
