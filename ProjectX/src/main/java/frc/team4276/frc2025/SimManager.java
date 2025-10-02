package frc.team4276.frc2025;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4276.frc2025.subsystems.hopper.HopperConstants;
import frc.team4276.frc2025.subsystems.superstructure.elevator.ElevatorConstants;
import org.littletonrobotics.junction.Logger;

public class SimManager {
  private SimManager() {}
  ;

  static {
    SmartDashboard.putBoolean("CoralSim/HasCoral", false);
  }

  private static final double climberRadsPerMotorRotation = 2 * Math.PI / (125 * 3);

  private static double elevatorMeasured = 0.0;
  private static double hopperLeftMeasured = 0.0;
  private static double hopperRightMeasured = 0.0;
  private static double climberMeasured = 0.0;

  private static double elevatorGoal = 0.0;
  private static double hopperGoal = 0.0;
  private static double climberGoal = 0.0;

  public static void addElevatorMeasuredObs(double position) {
    elevatorMeasured = position;
  }

  public static void addHopperLeftMeasuredObs(double position) {
    hopperLeftMeasured = position;
  }

  public static void addHopperRightMeasuredObs(double position) {
    hopperRightMeasured = position;
  }

  public static void addClimberMeasuredObs(double position) {
    climberMeasured = position;
  }

  public static void addElevatorGoalObs(double position) {
    elevatorGoal = position;
  }

  public static void addHopperGoalObs(double position) {
    hopperGoal = position;
  }

  public static void addClimberGoalObs(double position) {
    climberGoal = position;
  }

  public static void setHasCoral(boolean hasCoral) {
    SmartDashboard.putBoolean("CoralSim/HasCoral", hasCoral);
  }

  public static boolean hasCoral() {
    return SmartDashboard.getBoolean("CoralSim/HasCoral", false);
  }

  public static void periodic() {
    // Log 3D measured poses
    Logger.recordOutput(
        "Elevator/Mechanism3d/Measured",
        new Pose3d(
            new Translation3d(0.0, 0.0, ElevatorConstants.origin.getY() + elevatorMeasured),
            Rotation3d.kZero));

    Logger.recordOutput(
        "Hopper/Mechanism3d/Measured/Left",
        new Pose3d(
            Translation3d.kZero,
            new Rotation3d(
                0.0, 0.0, hopperLeftMeasured * HopperConstants.radsPerMotorRotation * -1.0)));
    Logger.recordOutput(
        "Hopper/Mechanism3d/Measured/Right",
        new Pose3d(
            Translation3d.kZero,
            new Rotation3d(0.0, 0.0, hopperRightMeasured * HopperConstants.radsPerMotorRotation)));

    Logger.recordOutput(
        "Climber/Mechanism3d/Measured",
        new Pose3d(
            Translation3d.kZero,
            new Rotation3d(0.0, climberMeasured * climberRadsPerMotorRotation, 0.0)));

    // Log 3D goal poses
    Logger.recordOutput(
        "Elevator/Mechanism3d/Goal",
        new Pose3d(
            new Translation3d(0.0, 0.0, ElevatorConstants.origin.getY() + elevatorGoal),
            Rotation3d.kZero));

    Logger.recordOutput(
        "Hopper/Mechanism3d/Goal/Left",
        new Pose3d(
            Translation3d.kZero,
            new Rotation3d(0.0, 0.0, hopperGoal * HopperConstants.radsPerMotorRotation * -1.0)));
    Logger.recordOutput(
        "Hopper/Mechanism3d/Goal/Right",
        new Pose3d(
            Translation3d.kZero,
            new Rotation3d(0.0, 0.0, hopperGoal * HopperConstants.radsPerMotorRotation)));

    Logger.recordOutput(
        "Climber/Mechanism3d/Goal",
        new Pose3d(
            Translation3d.kZero,
            new Rotation3d(0.0, climberGoal * climberRadsPerMotorRotation, 0.0)));

    Pose3d coralPose =
        hasCoral()
            ? new Pose3d(RobotState.getInstance().getEstimatedPose())
                .plus(new Transform3d(0.0, 0.0, 1.0, Rotation3d.kZero))
            : Pose3d.kZero;

    Logger.recordOutput("CoralSim/Poses", new Pose3d[] {coralPose});
  }
}
