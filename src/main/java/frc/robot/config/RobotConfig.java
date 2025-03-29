package frc.robot.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;

public record RobotConfig(
    String robotName,
    ElevatorConfig elevator,
    IntakeConfig intake,
    ClawConfig claw,
    SwerveConfig swerve,
    VisionConfig vision,
    ArmConfig arm,
    DeployConfig deploy,
    ClimberConfig climber,
    LightsConfig lights) {
  public record ElevatorConfig(
      String canBusName,
      int leftMotorId,
      int rightMotorId,
      TalonFXConfiguration leftMotorConfig,
      TalonFXConfiguration rightMotorConfig,
      double homingEndHeight,
      double homingCurrentThreshold,
      double minHeight,
      double maxHeight) {}

  public record IntakeConfig(
      String canBusName,
      int motorId,
      int candiId,
      Debouncer debouncer,
      TalonFXConfiguration motorConfig) {}

  public record ClawConfig(
      String canBusName,
      int motorId,
      int candiId,
      Debouncer debouncer,
      TalonFXConfiguration motorConfig) {}

  public record SwerveConfig(
      PhoenixPIDController snapController,
      boolean invertRotation,
      boolean invertX,
      boolean invertY) {}

  public record VisionConfig(
      int translationHistoryArraySize,
      double xyStdDev,
      double thetaStdDev,
      Pose3d robotPoseRelativeToCalibration,
      Pose3d leftBackLimelightPosition,
      Pose3d leftFrontLimelightPosition,
      Pose3d rightLimelightPosition,
      Pose3d gamePieceDetectionLimelightPosition) {}

  public record ArmConfig(
      String canBusName,
      int motorId,
      TalonFXConfiguration motorConfig,
      double homingPosition,
      double inchesFromCenter) {}

  public record ClimberConfig(
      String canBusName,
      int climbMotorId,
      int cancoderId,
      int grabMotorId,
      int canrangeId,
      double minAngle,
      double maxAngle,
      TalonFXConfiguration climbMotorConfig,
      CANcoderConfiguration cancoderConfig,
      TalonFXConfiguration grabMotorConfig,
      CANrangeConfiguration canRangeConfig) {}

  public record DeployConfig(
      String canBusName,
      int motorId,
      TalonFXConfiguration motorConfig,
      double minAngle,
      double maxAngle,
      double homingVoltage,
      double homingCurrentThreshold,
      double homingEndPosition) {}

  public record LightsConfig(String canBusName, int candleId) {}

  // TODO: Change this to false during events
  public static final boolean IS_DEVELOPMENT = true;
  public static final String SERIAL_NUMBER = System.getenv("serialnum");
  private static final String PRACTICE_BOT_SERIAL_NUMBER = "0329F344";
  public static final boolean IS_PRACTICE_BOT =
      SERIAL_NUMBER != null && SERIAL_NUMBER.equals(PRACTICE_BOT_SERIAL_NUMBER);

  public static RobotConfig get() {
    return IS_PRACTICE_BOT ? PracticeConfig.practiceBot : CompConfig.competitionBot;
  }
}
