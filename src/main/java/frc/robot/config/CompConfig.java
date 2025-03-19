package frc.robot.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
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
import frc.robot.config.RobotConfig.ClimberConfig;
import frc.robot.config.RobotConfig.DeployConfig;
import frc.robot.config.RobotConfig.ElevatorConfig;
import frc.robot.config.RobotConfig.IntakeConfig;
import frc.robot.config.RobotConfig.LightsConfig;
import frc.robot.config.RobotConfig.RollConfig;
import frc.robot.config.RobotConfig.SwerveConfig;
import frc.robot.config.RobotConfig.VisionConfig;
import frc.robot.config.RobotConfig.WristConfig;
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
                          .withKP(1.0)
                          .withKV(0)
                          .withKG(0.4)
                          .withGravityType(GravityTypeValue.Elevator_Static))
                  .withMotionMagic(
                      new MotionMagicConfigs()
                          .withMotionMagicAcceleration(275.0)
                          .withMotionMagicCruiseVelocity(275.0))
                  .withFeedback(
                      new FeedbackConfigs()
                          .withSensorToMechanismRatio(
                              1
                                  / ((14.0 / 50.0)
                                      * (36.0 / 50.0)
                                      * (2.0 / 1.0)
                                      * (Math.PI * 1.274)))),
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
                          .withKP(1.0)
                          .withKV(0)
                          .withKG(0.4)
                          .withGravityType(GravityTypeValue.Elevator_Static))
                  .withMotionMagic(
                      new MotionMagicConfigs()
                          .withMotionMagicAcceleration(275)
                          .withMotionMagicCruiseVelocity(275.0))
                  .withFeedback(
                      new FeedbackConfigs()
                          .withSensorToMechanismRatio(
                              1
                                  / ((14.0 / 50.0)
                                      * (36.0 / 50.0)
                                      * (2.0 / 1.0)
                                      * (Math.PI * 1.274)))),
              0,
              25,
              0,
              58,
              0.5),
          new IntakeConfig(
              RIO_CAN_NAME,
              20,
              21,
              26,
              true,
              new Debouncer(0.1, DebounceType.kBoth),
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
                          .withPeakReverseTorqueCurrent(80.0)),
              new TalonFXConfiguration()
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimit(80)
                          .withSupplyCurrentLimit(80))
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withInverted(InvertedValue.CounterClockwise_Positive)
                          .withNeutralMode(NeutralModeValue.Coast))
                  .withTorqueCurrent(
                      new TorqueCurrentConfigs()
                          .withPeakForwardTorqueCurrent(80.0)
                          .withPeakReverseTorqueCurrent(80.0)),
              90,
              0.66),
          new SwerveConfig(
              new PhoenixPIDController(5.75, 0, 0),
              true,
              true,
              true,
              new TalonFXConfiguration()
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimitEnable(true)
                          .withStatorCurrentLimit(80)
                          .withSupplyCurrentLimitEnable(true)
                          .withSupplyCurrentLimit(80))
                  .withOpenLoopRamps(
                      new OpenLoopRampsConfigs()
                          .withDutyCycleOpenLoopRampPeriod(0.01)
                          .withVoltageOpenLoopRampPeriod(0.01)
                          .withTorqueOpenLoopRampPeriod(0.01))
                  .withVoltage(
                      new VoltageConfigs().withPeakForwardVoltage(12).withPeakReverseVoltage(-12))
                  .withMotorOutput(
                      new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)),
              new TalonFXConfiguration()
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          // Swerve azimuth does not require much torque output, so we can set a
                          // relatively low stator current limit to help avoid brownouts without
                          // impacting performance.
                          .withStatorCurrentLimitEnable(true)
                          .withStatorCurrentLimit(50)
                          .withSupplyCurrentLimitEnable(true)
                          .withSupplyCurrentLimit(80))
                  .withVoltage(
                      new VoltageConfigs().withPeakForwardVoltage(12).withPeakReverseVoltage(-12))
                  .withMotorOutput(
                      new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))),
          new VisionConfig(4, 0.4, Double.MAX_VALUE),
          new WristConfig(
              RIO_CAN_NAME,
              22,
              new TalonFXConfiguration()
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withNeutralMode(NeutralModeValue.Brake)
                          .withInverted(InvertedValue.Clockwise_Positive))
                  .withSlot0(
                      new Slot0Configs()
                          .withKP(100.0)
                          .withKV(0.0)
                          .withKD(3.0)
                          .withKG(0.3)
                          .withGravityType(GravityTypeValue.Arm_Cosine))
                  // .withMotionMagic(
                  //     new MotionMagicConfigs()
                  //         .withMotionMagicAcceleration(10.0)
                  //         .withMotionMagicCruiseVelocity(150.0 / 7.0))
                  .withFeedback(
                      new FeedbackConfigs()
                          .withSensorToMechanismRatio((64.0 / 8.0) * (50.0 / 18.0) * (36.0 / 12.0)))
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withSupplyCurrentLimitEnable(true)
                          .withStatorCurrentLimitEnable(true)
                          .withSupplyCurrentLimit(30.0)
                          .withStatorCurrentLimit(60.0)),
              -56.4,
              156.5,
              18,
              -56.4),
          new RollConfig(
              RIO_CAN_NAME,
              23,
              new TalonFXConfiguration()
                  .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
                  .withFeedback(
                      new FeedbackConfigs()
                          .withSensorToMechanismRatio((60.0 / 8.0) * (60.0 / 15.0)))
                  .withSlot0(new Slot0Configs().withKP(50.0).withKV(0.0))
                  .withVoltage(
                      new VoltageConfigs().withPeakForwardVoltage(6.0).withPeakReverseVoltage(-6.0))
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimitEnable(true)
                          .withStatorCurrentLimit(20)
                          .withSupplyCurrentLimitEnable(true)
                          .withSupplyCurrentLimit(20)),
              -94.31,
              97.91,
              9,
              97.91),
          new DeployConfig(
              CANIVORE_NAME,
              999,
              new TalonFXConfiguration()
                  .withFeedback(
                      new FeedbackConfigs()
                          .withSensorToMechanismRatio((50 / 8) * (50 / 18) * (40 / 10)))
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
                          .withKP(1.0)
                          .withKV(0.0)
                          .withKG(0.0)
                          .withGravityType(GravityTypeValue.Arm_Cosine)),
              0.0,
              581,
              0.0,
              0.0,
              0.0), // TODO: get these numbers
          new ClimberConfig(
              CANIVORE_NAME,
              24,
              25,
              27,
              28,
              -55.0,
              100.0,
              new TalonFXConfiguration()
                  .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
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
          new LightsConfig(RIO_CAN_NAME, 18));

  private CompConfig() {}
}
