package frc.robot.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.filter.Debouncer;

public record RobotConfig(
    String robotName,
    ElevatorConfig elevator,
    IntakeConfig intake,
    ClawConfig claw,
    SwerveConfig swerve,
    VisionConfig vision,
    ArmConfig arm,
    RollConfig roll,
    DeployConfig deploy,
    ClimberConfig climber,
    LightsConfig lights) {
  public record ElevatorConfig(
      String canBusName,
      int leftMotorID,
      int rightMotorID,
      TalonFXConfiguration leftMotorConfig,
      TalonFXConfiguration rightMotorConfig,
      double homingEndHeight,
      double homingCurrentThreshold,
      double minHeight,
      double maxHeight,
      double tolerance) {}

  public record IntakeConfig(
      String canBusName,
      int motorID,
      int candiID,
      Debouncer debouncer,
      TalonFXConfiguration motorConfig) {}

  public record ClawConfig(
      String canBusName,
      int motorID,
      int candiID,
      Debouncer debouncer,
      TalonFXConfiguration motorConfig,
      double algaeHoldCurrent,
      double algaeHoldMaxDutyCycle) {}

  public record SwerveConfig(
      PhoenixPIDController snapController,
      boolean invertRotation,
      boolean invertX,
      boolean invertY,
      TalonFXConfiguration driveMotorConfig,
      TalonFXConfiguration steerMotorConfig) {}

  public record VisionConfig(
      int translationHistoryArraySize, double xyStdDev, double thetaStdDev) {}

  public record ArmConfig(
      String canBusName,
      int motorID,
      TalonFXConfiguration motorConfig,
      double minAngle,
      double maxAngle,
      double homingCurrentThreshold,
      double homingPosition) {}

  public record ClimberConfig(
      String canBusName,
      int climbMotorID,
      int cancoderID,
      int grabMotorID,
      int canrangeID,
      double minAngle,
      double maxAngle,
      TalonFXConfiguration climbMotorConfig,
      CANcoderConfiguration cancoderConfig,
      TalonFXConfiguration grabMotorConfig,
      CANrangeConfiguration canRangeConfig) {}

  public record RollConfig(
      String canBusName,
      int motorID,
      TalonFXConfiguration motorConfig,
      double minAngle,
      double maxAngle,
      double homingCurrentThreshold,
      double homingPosition) {}

  public record DeployConfig(
      String canBusName,
      int motorID,
      TalonFXConfiguration motorConfig,
      double minAngle,
      double maxAngle,
      double homingVoltage,
      double homingCurrent,
      double homingCurrentThreshold) {}

  public record LightsConfig(String canBusName, int candleID) {}

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
