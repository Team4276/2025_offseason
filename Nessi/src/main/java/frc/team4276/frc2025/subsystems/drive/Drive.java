package frc.team4276.frc2025.subsystems.drive;

import static frc.team4276.frc2025.subsystems.drive.DriveConstants.*;

import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.frc2025.Constants;
import frc.team4276.frc2025.RobotState;
import frc.team4276.util.dashboard.ElasticUI;
import frc.team4276.util.ios.GyroIO;
import frc.team4276.util.ios.GyroIOInputsAutoLogged;
import frc.team4276.util.ios.ModuleIO;
import frc.team4276.util.swerve.SwerveSetpointGenerator;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  public enum WantedState {
    TELEOP,
    TRAJECTORY,
    HEADING_ALIGN,
    AUTO_ALIGN,
    IDLE,
    CHARACTERIZATION
  }

  public enum SystemState {
    TELEOP,
    TRAJECTORY,
    HEADING_ALIGN,
    AUTO_ALIGN,
    IDLE,
    CHARACTERIZATION
  }

  private WantedState wantedState = WantedState.TELEOP;
  private SystemState systemState = SystemState.TELEOP;

  public enum DriveMode {
    /** Driving with input from driver joysticks. (Default) */
    TELEOP,

    /** Driving based on a trajectory. */
    TRAJECTORY,

    /** Driving with a heading on the field automatically. */
    HEADING_ALIGN,

    /** Driving to a location on the field automatically. */
    AUTO_ALIGN,

    /** Characterizing (modules oriented forwards, motor outputs supplied externally). */
    CHARACTERIZATION,

    /** Running wheel radius characterization routine (spinning in circle) */
    WHEEL_RADIUS_CHARACTERIZATION
  }

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveModulePosition[] lastModulePositions = null;
  private double lastTime = 0.0;

  private boolean useSetpointGenerator = true;
  private final SwerveSetpointGenerator swerveSetpointGenerator =
      new SwerveSetpointGenerator(driveConfig, maxSteerVelocity);
  private SwerveSetpoint prevSetpoint;
  private boolean disableTrajFF = true;
  private final double[] dummyForces = {0.0, 0.0, 0.0, 0.0};

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Start odometry thread
    SparkOdometryThread.getInstance().start();

    prevSetpoint =
        new SwerveSetpoint(getChassisSpeeds(), getModuleStates(), DriveFeedforwards.zeros(4));

    ElasticUI.putSwerveDrive(
        () -> getModuleStates(), () -> RobotState.getInstance().getEstimatedPose().getRotation());
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    if (DriverStation.isDisabled()) {
      // Stop moving when disabled
      for (var module : modules) {
        module.stop();
      }

      // Log empty setpoint states when disabled
      Logger.recordOutput("Drive/SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("Drive/SwerveStates/OptimizedSetpoints", new SwerveModuleState[] {});
      Logger.recordOutput("Drive/SwerveStates/Torques", new SwerveModuleState[] {});
      if (Constants.isTuning) {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
          states[i] = modules[i].getZeroHelperModuleState();
        }
        Logger.recordOutput("Drive/SwerveStates/ZeroHelper", states);
      }
    }

    updateOdom();

    systemState = handleStateTransition();
    Logger.recordOutput("Drive/SystemState", systemState);
    Logger.recordOutput("Drive/DesiredState", wantedState);
    applyState();

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && !Constants.isSim);
  }

  private void updateOdom() {
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
      }

      boolean includeMeasurement = true;
      if (lastModulePositions != null) {

        double dt = sampleTimestamps[i] - lastTime;
        for (int j = 0; j < modules.length; j++) {
          double velocity =
              (modulePositions[j].distanceMeters - lastModulePositions[j].distanceMeters) / dt;
          double omega =
              modulePositions[j].angle.minus(lastModulePositions[j].angle).getRadians() / dt;
          // Check if delta is too large
          if (Math.abs(omega) > DriveConstants.maxSpeed * 1.5
              || Math.abs(velocity) > DriveConstants.maxAngularSpeed * 1.5) {
            includeMeasurement = false;
            break;
          }
        }
      }

      // If delta isn't too large we can include the measurement.
      if (includeMeasurement) {
        lastModulePositions = modulePositions;
        RobotState.getInstance()
            .addOdometryObservation(
                sampleTimestamps[i],
                gyroInputs.connected ? gyroInputs.yawPosition : null,
                modulePositions);
        lastTime = sampleTimestamps[i];
        RobotState.getInstance().addDriveSpeeds(getChassisSpeeds());
      }
    }
  }

  private SystemState handleStateTransition() {
    return switch (wantedState) {
      case TELEOP -> SystemState.TELEOP;
      case TRAJECTORY -> SystemState.TRAJECTORY;
      case HEADING_ALIGN -> SystemState.HEADING_ALIGN;
      case AUTO_ALIGN -> SystemState.AUTO_ALIGN;
      case CHARACTERIZATION -> SystemState.CHARACTERIZATION;
      default -> SystemState.IDLE;
    };
  }

  private void applyState() {
    switch (systemState) {
      case TELEOP:
        break;

      case TRAJECTORY:
        break;

      case HEADING_ALIGN:
        break;

      case AUTO_ALIGN:
        break;

      case IDLE:
        break;

      case CHARACTERIZATION:
        break;
    }
  }

  public void runVelocity(ChassisSpeeds speeds, DriveMode mode) {

    // Calculate setpoints
    ChassisSpeeds setpointSpeeds;
    SwerveModuleState[] setpointStates;
    if (useSetpointGenerator
        && mode != DriveMode.TRAJECTORY
        && mode != DriveMode.TELEOP
        && mode != DriveMode.HEADING_ALIGN) {
      prevSetpoint = swerveSetpointGenerator.generateSetpoint(prevSetpoint, speeds, 0.02);
      setpointSpeeds = prevSetpoint.robotRelativeSpeeds();
      setpointStates = prevSetpoint.moduleStates();
    } else {
      setpointSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
      setpointStates = kinematics.toSwerveModuleStates(setpointSpeeds);
    }

    SwerveModuleState[] setpointTorques =
        new SwerveModuleState[] {
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState()
        };

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("Drive/SetpointSpeeds", setpointSpeeds);
    Logger.recordOutput("Drive/SwerveStates/OptimizedSetpoints", setpointStates);
    Logger.recordOutput("Drive/SwerveStates/Torques", setpointTorques);
    Logger.recordOutput(
        "Drive/SwerveStates/UnoptimizedSetpoints",
        kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(speeds, 0.02)));
    Logger.recordOutput("Drive/DesiredSpeeds", speeds);
    Logger.recordOutput("Drive/DriveMode", mode.toString());
  }

  public void runVelocity(ChassisSpeeds speeds, List<Vector<N2>> forces, DriveMode mode) {

    // Calculate setpoints
    ChassisSpeeds setpointSpeeds;
    SwerveModuleState[] setpointStates;
    if (useSetpointGenerator
        && mode != DriveMode.TRAJECTORY
        && mode != DriveMode.TELEOP
        && mode != DriveMode.HEADING_ALIGN) {
      prevSetpoint = swerveSetpointGenerator.generateSetpoint(prevSetpoint, speeds, 0.02);
      setpointSpeeds = prevSetpoint.robotRelativeSpeeds();
      setpointStates = prevSetpoint.moduleStates();
    } else {
      setpointSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
      setpointStates = kinematics.toSwerveModuleStates(setpointSpeeds);
    }

    SwerveModuleState[] setpointTorques =
        new SwerveModuleState[] {
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState()
        };

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      if (disableTrajFF) {
        modules[i].runSetpoint(setpointStates[i]);

      } else {
        modules[i].runSetpoint(setpointStates[i], forces.get(i));
      }
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("Drive/SetpointSpeeds", setpointSpeeds);
    Logger.recordOutput("Drive/SwerveStates/OptimizedSetpoints", setpointStates);
    Logger.recordOutput("Drive/SwerveStates/Torques", setpointTorques);
    Logger.recordOutput(
        "Drive/SwerveStates/UnoptimizedSetpoints",
        kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(speeds, 0.02)));
    Logger.recordOutput("Drive/DesiredSpeeds", speeds);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds(), DriveMode.TELEOP);
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "Drive/SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "Drive/MeasuredSpeeds")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }
}
