package frc.robot.arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.config.FeatureFlags;
import frc.robot.config.RobotConfig;
import frc.robot.util.MathHelpers;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.util.tuning.TunablePid;

public class ArmSubsystem extends StateMachine<ArmState> {
  public static final double ARM_LENGTH_METERS = Units.inchesToMeters(37.416);

  private static final double TOLERANCE = 2.0;
  private static final double NEAR_TOLERANCE = 10.0;
  private static final double CLIMBER_UNSAFE_ANGLE = 225.0;
  private final TalonFX motor;
  private double rawMotorAngle;
  private double motorAngle;
  private double motorCurrent;
  private double lowestSeenAngle = Double.POSITIVE_INFINITY;
  private double highestSeenAngle = Double.NEGATIVE_INFINITY;
  private double collisionAvoidanceGoal;
  private static final double MINIMUM_EXPECTED_HOMING_ANGLE_CHANGE = 90.0;
  private final StaticBrake brakeNeutralRequest = new StaticBrake();
  private final CoastOut coastNeutralRequest = new CoastOut();
  private final VelocityVoltage spinToWin = new VelocityVoltage(0.6);

  private final MotionMagicVoltage motionMagicRequest =
      new MotionMagicVoltage(0.0).withEnableFOC(false);

  // TODO: tune velocity
  private final PositionVoltage algaeFling =
      new PositionVoltage(Units.degreesToRotations(ArmState.ALGAE_FLING_SWING.getAngle()))
          .withVelocity(Units.degreesToRotations(90));

  public ArmSubsystem(TalonFX motor) {
    super(
        SubsystemPriority.ARM,
        RobotConfig.IS_PRACTICE_BOT ? ArmState.PRE_MATCH_HOMING : ArmState.HOLDING_UPRIGHT);
    motor.getConfigurator().apply(RobotConfig.get().arm().motorConfig());

    this.motor = motor;

    // In field calibration mode, boot arm to lower hardstop angle
    if (FeatureFlags.FIELD_CALIBRATION.getAsBoolean()) {
      motor.setPosition(Units.degreesToRotations(RobotConfig.get().arm().homingPosition()));
    }

    TunablePid.of("Arm", motor, RobotConfig.get().arm().motorConfig());

    DogLog.tunable(
        "Arm/ChangeEncoderPositionDeg",
        0.0,
        newPositionDeg -> motor.setPosition(Units.degreesToRotations(newPositionDeg)));
  }

  public void setState(ArmState newState) {
    switch (getState()) {
      case PRE_MATCH_HOMING -> {}
      default -> {
        setStateFromRequest(newState);
      }
    }
  }

  public double getAngle() {
    return motorAngle;
  }

  public void setCollisionAvoidanceGoal(double angle, boolean climberRisky) {

    int wrap = (int) rawMotorAngle / 360;
    double forwardSolution = (wrap * 360) + angle;
    double backwardSolution = (wrap * 360) - angle;

    if (climberRisky) {
      if (CLIMBER_UNSAFE_ANGLE > rawMotorAngle && CLIMBER_UNSAFE_ANGLE < forwardSolution) {
        collisionAvoidanceGoal = backwardSolution;
      }
      collisionAvoidanceGoal = forwardSolution;
    }
    collisionAvoidanceGoal =
        Math.min(
            Math.abs(backwardSolution - rawMotorAngle), Math.abs(forwardSolution - rawMotorAngle));
    DogLog.log("Arm/CollisionAvoidanceGoalAngle", collisionAvoidanceGoal);
  }

  public boolean atGoal() {
    return switch (getState()) {
      default -> MathUtil.isNear(getState().getAngle(), motorAngle, TOLERANCE, -180, 180);
      case ALGAE_FLING_SWING -> motorAngle >= getState().getAngle();
      case PRE_MATCH_HOMING, COLLISION_AVOIDANCE -> false;
    };
  }

  public boolean nearGoal() {
    return switch (getState()) {
      default -> MathUtil.isNear(getState().getAngle(), motorAngle, NEAR_TOLERANCE, -180, 180);
      case PRE_MATCH_HOMING, COLLISION_AVOIDANCE -> false;
    };
  }

  @Override
  protected void collectInputs() {
    rawMotorAngle = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
    motorAngle = MathHelpers.angleModulus(rawMotorAngle);
    if (DriverStation.isDisabled()) {
      lowestSeenAngle = Math.min(lowestSeenAngle, rawMotorAngle);
      highestSeenAngle = Math.max(highestSeenAngle, rawMotorAngle);
    }

    motorCurrent = motor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  protected void afterTransition(ArmState newState) {
    switch (newState) {
      case COLLISION_AVOIDANCE -> {
        motor.setControl(
            motionMagicRequest.withPosition(Units.degreesToRotations(collisionAvoidanceGoal)));
      }
      case PRE_MATCH_HOMING -> {
        motor.setControl(coastNeutralRequest);
      }
      case SPIN_TO_WIN -> {
        motor.setControl(spinToWin);
      }
      case ALGAE_FLING_SWING -> {
        motor.setControl(algaeFling);
      }
      default -> {
        motor.setControl(
            motionMagicRequest.withPosition(Units.degreesToRotations(newState.getAngle())));
      }
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    DogLog.log("Arm/StatorCurrent", motorCurrent);
    DogLog.log("Arm/AppliedVoltage", motor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Arm/Angle", motorAngle);
    DogLog.log("Arm/AtGoal", atGoal());
    if (getState() == ArmState.PRE_MATCH_HOMING) {
      if (rangeOfMotionGood()) {
        if (DriverStation.isEnabled()) {
          motor.setPosition(
              Units.degreesToRotations(
                  RobotConfig.get().arm().homingPosition() + (rawMotorAngle - lowestSeenAngle)));

          setStateFromRequest(ArmState.HOLDING_UPRIGHT);
        } else {
          motor.setControl(brakeNeutralRequest);
        }
      }
    }
    if (DriverStation.isDisabled()) {
      DogLog.log("Arm/LowestAngle", lowestSeenAngle);
      DogLog.log("Arm/HighestAngle", highestSeenAngle);
    }
    if (rangeOfMotionGood()) {
      DogLog.clearFault("Arm not seen range of motion");
    } else if (RobotConfig.IS_PRACTICE_BOT) {
      DogLog.logFault("Arm not seen range of motion", AlertType.kWarning);
    }

    switch (getState()) {
      case COLLISION_AVOIDANCE -> {
        motor.setControl(
            motionMagicRequest.withPosition(Units.degreesToRotations(collisionAvoidanceGoal)));
      }
      default -> {}
    }
  }

  public boolean rangeOfMotionGood() {
    return Math.abs(highestSeenAngle - lowestSeenAngle) > MINIMUM_EXPECTED_HOMING_ANGLE_CHANGE;
  }

  private final TalonFXConfiguration simMotorConfig = new TalonFXConfiguration();
  private TrapezoidProfile.Constraints simConstraints;
  private boolean simDidInit = false;

  @Override
  public void simulationPeriodic() {
    if (getState() == ArmState.PRE_MATCH_HOMING) {
      motor.setPosition(0);
      setStateFromRequest(ArmState.HOLDING_UPRIGHT);
    }

    if (!simDidInit) {
      motor.getConfigurator().refresh(simMotorConfig);

      simConstraints =
          new TrapezoidProfile.Constraints(
              simMotorConfig.MotionMagic.MotionMagicCruiseVelocity,
              simMotorConfig.MotionMagic.MotionMagicAcceleration);

      simDidInit = true;
    }

    if (DriverStation.isDisabled()) {
      return;
    }

    var currentState =
        new TrapezoidProfile.State(
            motor.getPosition().getValueAsDouble(), motor.getVelocity().getValueAsDouble());
    var wantedState =
        new TrapezoidProfile.State(motor.getClosedLoopReference().getValueAsDouble(), 0);

    var predictedState =
        new TrapezoidProfile(simConstraints).calculate(0.02, currentState, wantedState);

    var motorSim = motor.getSimState();

    motorSim.setRawRotorPosition(
        predictedState.position * simMotorConfig.Feedback.SensorToMechanismRatio);

    motorSim.setRotorVelocity(
        predictedState.velocity * simMotorConfig.Feedback.SensorToMechanismRatio);
  }
}
