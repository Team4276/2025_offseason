package frc.team4276.frc2025.subsystems.intake;

import edu.wpi.first.math.util.Units;

public class IntakeConstants {
    public static final double pivotStowPosition = Units.degreesToRadians(90.0);
    public static final double pivotIntakePosition = Units.degreesToRadians(0.0);
    public static final double pivotStagePosition = Units.degreesToRadians(90.0);
    public static final double pivotHoldPosition = Units.degreesToRadians(90.0);
    public static final double pivotClearPosition = Units.degreesToRadians(90.0);
    public static final double pivotScorePosition = Units.degreesToRadians(90.0);
    public static final double pivotDeployPosition = Units.degreesToRadians(0.0);
    public static final double pivotEjectPosition = Units.degreesToRadians(45.0);
    
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
    public static final double maxAccel = 0.0;
    public static final double cruiseVel = 0.0;

    public static final double encoderOffset = 0.0;
}
