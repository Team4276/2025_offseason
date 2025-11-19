package frc.team4276.frc2025.subsystems.drive;

import static frc.team4276.frc2025.subsystems.drive.DriveConstants.*;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.frc2025.Constants;
import frc.team4276.lib.AllianceFlipUtil;
import frc.team4276.lib.hid.JoystickOutputController;
import frc.team4276.frc2025.RobotState;

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

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR

  private SwerveModulePosition[] lastModulePositions = null;
  private double lastTime = 0.0;

  private final JoystickOutputController controller;

  public enum DriveSpeedScalar {
    DEFAULT(1.0, 0.65),
    DEMO(0.1, 0.1);

    private final double linearVelocityScalar;
    private final double angularVelocityScalar;

    private DriveSpeedScalar(double linearVelocityScalar, double angularVelocityScalar) {
      this.linearVelocityScalar = linearVelocityScalar;
      this.angularVelocityScalar = angularVelocityScalar;
    }

    public double linearVelocityScalar() {
      return linearVelocityScalar;
    }

    public double angularVelocityScalar() {
      return angularVelocityScalar;
    }
  }

  private DriveSpeedScalar driveSpeedScalar = Constants.isDemo ? DriveSpeedScalar.DEMO : DriveSpeedScalar.DEFAULT;

  public Drive(
      JoystickOutputController controller,
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.controller = controller;
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Start odometry thread
    SparkOdometryThread.getInstance().start();
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

  }

  private void updateOdom() {
    double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
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
          double velocity = (modulePositions[j].distanceMeters - lastModulePositions[j].distanceMeters) / dt;
          double omega = modulePositions[j].angle.minus(lastModulePositions[j].angle).getRadians() / dt;
          // Check if delta is too large
          if (Math.abs(velocity) > DriveConstants.maxVelocityMPS * 1.5
              || Math.abs(omega) > DriveConstants.maxAngularVelocity * 1.5) {
            includeMeasurement = false;
            break;
          }
        }
      }
      Logger.recordOutput("Drive/lastMeasurementIncluded", includeMeasurement);
      // If delta isn't too large we can include the measurement.
      if (includeMeasurement) {
        lastModulePositions = modulePositions;
        RobotState.getInstance()
            .addOdometryObservation(
                sampleTimestamps[i],
                gyroInputs.connected ? gyroInputs.yawPosition : null,
                modulePositions);
        lastTime = sampleTimestamps[i];
        RobotState.getInstance().addDriveSpeeds(kinematics.toChassisSpeeds(getModuleStates()));
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
    ChassisSpeeds requestedSpeeds = new ChassisSpeeds();

    Pose2d currentPose = RobotState.getInstance().getEstimatedPose();

    Logger.recordOutput("RobotState/EstimatedPose", currentPose);
    Logger.recordOutput(
        "RobotState/EstimatedOdomPose", RobotState.getInstance().getEstimatedOdomPose());

    switch (systemState) {
      default:
        break;

      case TELEOP:
        requestedSpeeds = getJoystickRequestedSpeeds();

        break;

      case TRAJECTORY:

        break;

      case HEADING_ALIGN:
        break;

      case AUTO_ALIGN:

        break;

      case CHARACTERIZATION:
        break;
    }

    requestedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(requestedSpeeds, currentPose.getRotation());

    SwerveModuleState[] setpointStates;
    ChassisSpeeds setpointSpeeds;

    setpointSpeeds = ChassisSpeeds.discretize(requestedSpeeds, 0.02);
    setpointStates = kinematics.toSwerveModuleStates(setpointSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxVelocityMPS);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("Drive/RequestedSpeeds", requestedSpeeds);
    Logger.recordOutput("Drive/SetpointSpeeds", setpointSpeeds);
    Logger.recordOutput(
        "Drive/SwerveStates/UnoptimizedSetpoints",
        kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(requestedSpeeds, 0.02)));
    Logger.recordOutput("Drive/SwerveStates/OptimizedSetpoints", setpointStates);
  }

  private ChassisSpeeds getJoystickRequestedSpeeds() {
    double linearMagnitude = Math.hypot(-controller.getLeftWithDeadband().y, -controller.getLeftWithDeadband().x);

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    Translation2d linearVelocity = Translation2d.kZero;

    if (linearMagnitude > 1e-6) {
      linearVelocity = new Translation2d(
          linearMagnitude,
          new Rotation2d(
              controller.getLeftWithDeadband().y, controller.getLeftWithDeadband().x))
          .times(driveSpeedScalar.linearVelocityScalar);
    }

    // Square rotation value for more precise control
    double omega = Math.copySign(
        controller.getRightWithDeadband().x * controller.getRightWithDeadband().x,
        -controller.getRightWithDeadband().x)
        * driveSpeedScalar.angularVelocityScalar;

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        new ChassisSpeeds(
            linearVelocity.getX() * DriveConstants.maxVelocityMPS,
            linearVelocity.getY() * DriveConstants.maxVelocityMPS,
            omega * DriveConstants.maxAngularVelocity),
        AllianceFlipUtil.apply(Rotation2d.k180deg));
  }

  /**
   * Returns the module states (turn angles and drive velocities) for all of the
   * modules.
   */
  @AutoLogOutput(key = "Drive/SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

}
