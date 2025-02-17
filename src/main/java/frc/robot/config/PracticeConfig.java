package frc.robot.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import frc.robot.config.RobotConfig.ClimberConfig;
import frc.robot.config.RobotConfig.ElevatorConfig;
import frc.robot.config.RobotConfig.IntakeConfig;
import frc.robot.config.RobotConfig.LightsConfig;
import frc.robot.config.RobotConfig.RollConfig;
import frc.robot.config.RobotConfig.SwerveConfig;
import frc.robot.config.RobotConfig.VisionConfig;
import frc.robot.config.RobotConfig.WristConfig;
import frc.robot.generated.PracticeBotTunerConstants;
import frc.robot.util.ProfiledPhoenixPIDController;

class PracticeConfig {
  private static final String CANIVORE_NAME = PracticeBotTunerConstants.kCANBus.getName();
  private static final String RIO_CAN_NAME = "rio";

  public static final RobotConfig practiceBot =
      new RobotConfig(
          "practice",
          new ElevatorConfig(
              CANIVORE_NAME,
              15,
              16,
              // Sensor to mechanism ratio is the gear ratio multiplied by the sprocket circumfrence
              new TalonFXConfiguration()
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
                          .withMotionMagicAcceleration(250.0)
                          .withMotionMagicCruiseVelocity(250.0))
                  .withFeedback(
                      new FeedbackConfigs()
                          .withSensorToMechanismRatio(
                              1
                                  / ((14.0 / 50.0)
                                      * (36.0 / 50.0)
                                      * (2.0 / 1.0)
                                      * (Math.PI * 1.274)))),
              new TalonFXConfiguration()
                  .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
                  .withSlot0(
                      new Slot0Configs()
                          .withKP(1.0)
                          .withKV(0)
                          .withKG(0.4)
                          .withGravityType(GravityTypeValue.Elevator_Static))
                  .withMotionMagic(
                      new MotionMagicConfigs()
                          .withMotionMagicAcceleration(250)
                          .withMotionMagicCruiseVelocity(250.0))
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
              new Debouncer(0.1, DebounceType.kBoth),
              new Debouncer(0.1, DebounceType.kBoth),
              new TalonFXConfiguration()
                  .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(15))
                  .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(20))
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withInverted(InvertedValue.Clockwise_Positive)
                          .withNeutralMode(NeutralModeValue.Coast)),
              new TalonFXConfiguration()
                  .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(15))
                  .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(20))
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withInverted(InvertedValue.CounterClockwise_Positive)
                          .withNeutralMode(NeutralModeValue.Coast))),
          new SwerveConfig(
              new ProfiledPhoenixPIDController(10, 0, 1, Double.MAX_VALUE),
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
                  .withMotionMagic(
                      new MotionMagicConfigs()
                          .withMotionMagicAcceleration(5)
                          .withMotionMagicCruiseVelocity(10.0))
                  .withFeedback(
                      new FeedbackConfigs()
                          .withSensorToMechanismRatio((64.0 / 8.0) * (50.0 / 18.0) * (36.0 / 12.0)))
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withSupplyCurrentLimitEnable(true)
                          .withStatorCurrentLimitEnable(true)
                          .withSupplyCurrentLimit(25.0)
                          .withStatorCurrentLimit(40.0)),
              -58.7,
              156.5,
              18,
              -58.7),
          new RollConfig(
              RIO_CAN_NAME,
              23,
              new TalonFXConfiguration()
                  .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
                  .withFeedback(
                      new FeedbackConfigs()
                          .withSensorToMechanismRatio((60.0 / 8.0) * (60.0 / 15.0)))
                  .withSlot0(new Slot0Configs().withKP(50).withKV(0))
                  .withVoltage(
                      new VoltageConfigs().withPeakForwardVoltage(4).withPeakReverseVoltage(-4))
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimitEnable(true)
                          .withStatorCurrentLimit(10)
                          .withSupplyCurrentLimitEnable(true)
                          .withSupplyCurrentLimit(10)),
              9,
              95),
          new ClimberConfig(
              CANIVORE_NAME,
              24,
              25,
              new TalonFXConfiguration()
                  .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
                  .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(125.0))
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimitEnable(true)
                          .withStatorCurrentLimit(60)
                          .withSupplyCurrentLimitEnable(true)
                          .withSupplyCurrentLimit(60)),
              new CANcoderConfiguration()
                  .withMagnetSensor(
                      new MagnetSensorConfigs()
                          .withMagnetOffset(0.076416)
                          .withSensorDirection(SensorDirectionValue.Clockwise_Positive))),
          new LightsConfig(RIO_CAN_NAME, 18));

  private PracticeConfig() {}
}
