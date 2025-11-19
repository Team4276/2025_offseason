package frc.team4276.frc2025.subsystems.elevator;

import edu.wpi.first.math.util.Units;
import frc.team4276.frc2025.Ports;
import frc.team4276.lib.dashboard.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public class ElevatorConstants {
  public static final LoggedTunableNumber stow =
      new LoggedTunableNumber("Elevator/StowPosition", Units.inchesToMeters(0.5));
  public static final LoggedTunableNumber intake =
      new LoggedTunableNumber("Elevator/IntakePosition", Units.inchesToMeters(0.0));
  public static final LoggedTunableNumber unjam =
      new LoggedTunableNumber("Elevator/UnjamPosition", Units.inchesToMeters(15.0));
  public static final LoggedTunableNumber l1 =
      new LoggedTunableNumber("Elevator/L1Position", Units.inchesToMeters(0.0));
  public static final LoggedTunableNumber l2 =
      new LoggedTunableNumber("Elevator/L2Position", Units.inchesToMeters(6.85));
  public static final LoggedTunableNumber l3 =
      new LoggedTunableNumber("Elevator/L3Position", Units.inchesToMeters(22.5));
  public static final LoggedTunableNumber lowAlgae =
      new LoggedTunableNumber("Elevator/LoAlgae", Units.inchesToMeters(10.0));
  public static final LoggedTunableNumber highAlgae =
      new LoggedTunableNumber("Elevator/HiAlgae", Units.inchesToMeters(17.32));
  public static final LoggedTunableNumber algaeChop =
      new LoggedTunableNumber("Elevator/AlgaeChop", Units.inchesToMeters(0.0));
  public static final LoggedTunableNumber climb =
      new LoggedTunableNumber("Elevator/Climb", Units.inchesToMeters(0.0));

  public enum ElevatorPosition {
    STOW(stow),
    INTAKE(intake),
    UNJAM(unjam),
    L1(l1),
    L2(l2),
    L3(l3),
    LOW_ALGAE(lowAlgae),
    HIGH_ALGAE(highAlgae),
    ALGAE_CHOP(algaeChop),
    CLIMB(climb);

    private final DoubleSupplier elevatorSetpointSupplier;

    private ElevatorPosition(DoubleSupplier elevatorSetpointSupplier) {
      this.elevatorSetpointSupplier = elevatorSetpointSupplier;
    }

    public double getPositionMetres() {
      return elevatorSetpointSupplier.getAsDouble();
    }

    public double getPositionRotations(){
        return getPositionMetres() * gearRatio / drumCircumference;
    }
  }

  public static final int leaderId = Ports.ELEVATOR_LEADER;
  public static final int followerId = Ports.ELEVATOR_FOLLOWER;

  public static final boolean invertLeader = true;
  public static final boolean invertFollower = true;

  public static final int currentLimit = 50;

  public static final double drumCircumference = Units.inchesToMeters(0.25 * 22); // m
  public static final double drumDiameter = drumCircumference / Math.PI; // m
  public static final double encoderPositionFactor = 1.0;
  public static final double encoderVelocityFactor = 1.0 / 60.0;
  public static final boolean invertEncoder = true;

  public static final double kp = 0.1;
  public static final double ki = 0.0;
  public static final double kd = 0.0;

  public static final double tolerance = 0.01; // m

  public static final double gearRatio = 5.0;

  public static final double maxVelMps = 1.5;
  public static final double maxVelRps = maxVelMps * gearRatio / drumCircumference;
  public static final double maxAccelMpss = 20.0;
  public static final double maxAccelRpss = maxAccelMpss * gearRatio / drumCircumference;

  public static final double kS = 0.0;
  public static final double kV = 4.38;
  public static final double kG = 0.52;
  public static final double kA = 0.08;

  public static final int readFreq = 50;

  public static final double minInput = 0.0; // m
  public static final double maxInput = Units.inchesToMeters(25.5); // m

}
