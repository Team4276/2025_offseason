package frc.team4276.frc2025.subsystems.intake;

import edu.wpi.first.math.util.Units;
import frc.team4276.lib.dashboard.LoggedTunableNumber;

public class IntakeConstants {
    public static final double pivotStowPosition = Units.degreesToRadians(135.0);
    public static final double pivotTuckPosition = Units.degreesToRadians(135.0);
    public static final double pivotIntakePosition = Units.degreesToRadians(0.0);
    public static final double pivotStagePosition = Units.degreesToRadians(160.0);
    public static final double pivotHoldPosition = Units.degreesToRadians(135.0);
    public static final double pivotClearPosition = Units.degreesToRadians(110.0);
    public static final double pivotScorePosition = Units.degreesToRadians(110.0);
    public static final double pivotDeployPosition = Units.degreesToRadians(0.0);
    public static final double pivotEjectPosition = Units.degreesToRadians(60.0);
    
    public static final double rollerPassiveEjectVolts = 1.0;
    public static final double rollerIntakeVolts = -5.0;
    public static final double rollerStageEjectVolts = 5.0;
    public static final double rollerHoldVolts = 0.0;
    public static final double rollerScoreVolts = 5.0;
    public static final double rollerEjectVolts = 5.0;

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kS = 0.0;

    public static final double motorRotationsPerRad = (25.0 / 1.0) * (64.0 / 30.0) / (2 * Math.PI);

    public static final double maxAccel = Units.degreesToRadians(180.0) * motorRotationsPerRad;
    public static final double cruiseVel = Units.degreesToRadians(90.0) * motorRotationsPerRad;

    public static final double encoderOffset = 0.0 / (2 * Math.PI); //TODO: check and be safe :D
    public static final double positionTolerance = Units.degreesToRadians(1.0);

    public static final LoggedTunableNumber hasCoralTripCurrent = new LoggedTunableNumber("Intake/HasCoralTripCurrent", 10.0);
    public static final LoggedTunableNumber hasCoralDebounceTime = new LoggedTunableNumber("Intake/HasCoralDebounceTime", 0.25);
}
