package frc.robot.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.config.RobotConfig.ArmConfig;
import frc.robot.config.RobotConfig.ClawConfig;
import frc.robot.config.RobotConfig.ClimberConfig;
import frc.robot.config.RobotConfig.DeployConfig;
import frc.robot.config.RobotConfig.ElevatorConfig;
import frc.robot.config.RobotConfig.IntakeConfig;
import frc.robot.config.RobotConfig.LightsConfig;
import frc.robot.config.RobotConfig.SwerveConfig;
import frc.robot.config.RobotConfig.VisionConfig;
import frc.robot.generated.CompBotTunerConstants;

class CompConfig {
  private static final String CANIVORE_NAME = CompBotTunerConstants.kCANBus.getName();
  private static final String RIO_CAN_NAME = "rio";

  public static final RobotConfig competitionBot =
      new RobotConfig(
          "competition",
          new ElevatorConfig(
              CANIVORE_NAME,
              15,
              16,
              1.274,
              // Sensor to mechanism ratio is the gear ratio multiplied by the sprocket circumfrence
              new TalonFXConfiguration()
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimit(60)
                          .withSupplyCurrentLimit(40))
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withInverted(InvertedValue.Clockwise_Positive)
                          .withNeutralMode(NeutralModeValue.Brake))
                  .withSlot0(
                      new Slot0Configs()
                          .withKP(0.0)
                          .withKV(0)
                          .withKG(0.0)
                          .withGravityType(GravityTypeValue.Elevator_Static))
                  .withMotionMagic(
                      new MotionMagicConfigs()
                          .withMotionMagicAcceleration(0.0)
                          .withMotionMagicCruiseVelocity(0.0))
                  .withFeedback(
                      new FeedbackConfigs()
                          .withSensorToMechanismRatio(1.0 / ((8.0 / 40.0) * (2.0 / 1.0)))),
              new TalonFXConfiguration()
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimit(60)
                          .withSupplyCurrentLimit(40))
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withInverted(InvertedValue.CounterClockwise_Positive)
                          .withNeutralMode(NeutralModeValue.Brake))
                  .withSlot0(
                      new Slot0Configs()
                          .withKP(0.0)
                          .withKV(0)
                          .withKG(0.0)
                          .withGravityType(GravityTypeValue.Elevator_Static))
                  .withMotionMagic(
                      new MotionMagicConfigs()
                          .withMotionMagicAcceleration(0)
                          .withMotionMagicCruiseVelocity(0.0))
                  .withFeedback(
                      new FeedbackConfigs()
                          .withSensorToMechanismRatio(1.0 / ((8.0 / 40.0) * (2.0 / 1.0)))),
              0,
              25,
              0,
              58,
              0.5),
          new IntakeConfig(
              CANIVORE_NAME,
              25,
              26,
              new Debouncer(0.1, DebounceType.kBoth),
              new TalonFXConfiguration()
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimit(15)
                          .withSupplyCurrentLimit(20))
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withInverted(InvertedValue.CounterClockwise_Positive)
                          .withNeutralMode(NeutralModeValue.Coast))
                  .withTorqueCurrent(
                      new TorqueCurrentConfigs()
                          .withPeakForwardTorqueCurrent(70.0)
                          .withPeakReverseTorqueCurrent(70.0))),
          new ClawConfig(
              RIO_CAN_NAME,
              18,
              27,
              new Debouncer(0.1, DebounceType.kBoth),
              new TalonFXConfiguration()
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimit(80)
                          .withSupplyCurrentLimit(80))
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withInverted(InvertedValue.Clockwise_Positive)
                          .withNeutralMode(NeutralModeValue.Coast))
                  .withTorqueCurrent(
                      new TorqueCurrentConfigs()
                          .withPeakForwardTorqueCurrent(80.0)
                          .withPeakReverseTorqueCurrent(80.0))),
          new SwerveConfig(new PhoenixPIDController(5.75, 0, 0), true, true, true),
          new VisionConfig(
              4,
              0.05,
              0.1,
              // Translation: Positive X = Forward, Positive Y = Left, Positive Z = Up
              // Rotation: Positive X = Roll Right, Positive Y = Pitch Down, Positive Z = Yaw Left
              new Pose3d(
                  0.0,
                  Units.inchesToMeters(-57.128),
                  Units.inchesToMeters(-49.00),
                  new Rotation3d(0.0, 0.0, 0.0)),
              new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)),
              new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)),
              new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0))),
          new ArmConfig(
              RIO_CAN_NAME,
              19,
              new TalonFXConfiguration()
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withNeutralMode(NeutralModeValue.Brake)
                          .withInverted(InvertedValue.Clockwise_Positive))
                  .withSlot0(
                      new Slot0Configs()
                          .withKP(0.0)
                          .withKV(0.0)
                          .withKD(0.0)
                          .withKG(0.0)
                          .withGravityType(GravityTypeValue.Arm_Cosine))
                  // .withMotionMagic(
                  //     new MotionMagicConfigs()
                  //         .withMotionMagicAcceleration(10.0)
                  //         .withMotionMagicCruiseVelocity(150.0 / 7.0))
                  .withFeedback(
                      new FeedbackConfigs()
                          .withSensorToMechanismRatio((64.0 / 8.0) * (90.0 / 10.0)))
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withSupplyCurrentLimitEnable(true)
                          .withStatorCurrentLimitEnable(true)
                          .withSupplyCurrentLimit(30.0)
                          .withStatorCurrentLimit(60.0)),
              -56.4,
              6.615),
          new DeployConfig(
              CANIVORE_NAME,
              20,
              new TalonFXConfiguration()
                  .withFeedback(
                      new FeedbackConfigs()
                          .withSensorToMechanismRatio((50.0 / 8.0) * (50.0 / 18.0) * (40.0 / 10.0)))
                  .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
                  .withMotionMagic(
                      new MotionMagicConfigs()
                          .withMotionMagicAcceleration(0.0)
                          .withMotionMagicCruiseVelocity(0.0))
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimit(60)
                          .withStatorCurrentLimit(60))
                  .withSlot0(
                      new Slot0Configs()
                          .withKP(0.0)
                          .withKV(0.0)
                          .withKG(0.0)
                          .withGravityType(GravityTypeValue.Arm_Cosine)),
              0.0,
              581,
              0.0,
              0.0), // TODO: get these numbers
          new ClimberConfig(
              CANIVORE_NAME,
              21,
              22,
              23,
              24,
              -55.0,
              100.0,
              new TalonFXConfiguration()
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withNeutralMode(NeutralModeValue.Brake)
                          .withPeakReverseDutyCycle(0))
                  .withVoltage(new VoltageConfigs().withPeakReverseVoltage(0))
                  .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(75.0))
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimitEnable(true)
                          .withStatorCurrentLimit(60)
                          .withSupplyCurrentLimitEnable(true)
                          .withSupplyCurrentLimit(60)),
              new CANcoderConfiguration()
                  .withMagnetSensor(
                      new MagnetSensorConfigs()
                          .withMagnetOffset(-0.444580078125)
                          .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                          .withAbsoluteSensorDiscontinuityPoint(0.5)),
              new TalonFXConfiguration()
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimitEnable(true)
                          .withStatorCurrentLimit(35)
                          .withSupplyCurrentLimitEnable(true)
                          .withSupplyCurrentLimit(35)),
              new CANrangeConfiguration()
                  .withProximityParams(new ProximityParamsConfigs().withProximityThreshold(0.06))),
          new LightsConfig(RIO_CAN_NAME, 17));

  private CompConfig() {}
}
