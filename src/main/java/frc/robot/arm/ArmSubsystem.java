package frc.robot.arm;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.config.FeatureFlags;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class ArmSubsystem extends StateMachine<ArmState> {
  private final TalonFX motor;
  private double motorAngle;
  private double motorCurrent;
  private double lowestSeenAngle = Double.MAX_VALUE;
  private double highestSeenAngle = Double.MIN_VALUE;
  private double collisionAvoidanceGoal;
  private static final double MINIMUM_EXPECTED_HOMING_ANGLE_CHANGE = 120.0;
  private final StaticBrake brakeNeutralRequest = new StaticBrake();
  private final CoastOut coastNeutralRequest = new CoastOut();

  private double averageMotorCurrent;
  private LinearFilter linearFilter = LinearFilter.movingAverage(5);

  // private final MotionMagicVoltage motionMagicRequest =
  //     new MotionMagicVoltage(0.0).withEnableFOC(false);

  private final PositionVoltage pidRequest = new PositionVoltage(0).withEnableFOC(false);

  public ArmSubsystem(TalonFX motor) {
    super(SubsystemPriority.ARM, ArmState.PRE_MATCH_HOMING);
    motor.getConfigurator().apply(RobotConfig.get().arm().motorConfig());

    this.motor = motor;

    // In field calibration mode, boot arm to lower hardstop angle
    if (FeatureFlags.FIELD_CALIBRATION.getAsBoolean()) {
      motor.setPosition(Units.degreesToRotations(RobotConfig.get().arm().homingPosition()));
    }
  }

  public void setState(ArmState newState) {
    switch (getState()) {
      case PRE_MATCH_HOMING -> {
        if (newState == ArmState.MID_MATCH_HOMING) {
          setStateFromRequest(newState);
        }
      }
      case MID_MATCH_HOMING -> {}
      default -> {
        setStateFromRequest(newState);
      }
    }
  }

  public double getAngle() {
    return motorAngle;
  }

  public void setCollisionAvoidanceGoal(double angle) {
    collisionAvoidanceGoal = angle;
    DogLog.log("Arm/CollisionAvoidanceGoalAngle", collisionAvoidanceGoal);
  }

  public boolean atGoal() {
    return switch (getState()) {
      default -> MathUtil.isNear(getState().angle, motorAngle, 2);
      case COLLISION_AVOIDANCE -> MathUtil.isNear(collisionAvoidanceGoal, motorAngle, 1);
      case PRE_MATCH_HOMING -> true;
    };
  }

  @Override
  protected void collectInputs() {
    motorAngle = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
    if (DriverStation.isDisabled()) {
      lowestSeenAngle = Math.min(lowestSeenAngle, motorAngle);
      highestSeenAngle = Math.max(highestSeenAngle, motorAngle);
    }

    motorCurrent = motor.getStatorCurrent().getValueAsDouble();
    averageMotorCurrent = linearFilter.calculate(motorCurrent);
  }

  @Override
  protected void afterTransition(ArmState newState) {
    switch (newState) {
      case MID_MATCH_HOMING -> {
        motor.setVoltage(-0.5);
      }
      case COLLISION_AVOIDANCE -> {
        motor.setControl(
            pidRequest.withPosition(Units.degreesToRotations(clamp(collisionAvoidanceGoal))));
      }
      case PRE_MATCH_HOMING -> {
        motor.setControl(coastNeutralRequest);
      }
      default -> {
        motor.setControl(pidRequest.withPosition(Units.degreesToRotations(clamp(newState.angle))));
      }
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    DogLog.log("Arm/StatorCurrent", motorCurrent);
    DogLog.log("Arm/AverageStatorCurrent", averageMotorCurrent);
    DogLog.log("Arm/AppliedVoltage", motor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Arm/Angle", motorAngle);
    DogLog.log("Arm/AtGoal", atGoal());
    if (DriverStation.isDisabled()) {
      DogLog.log("Arm/LowestAngle", lowestSeenAngle);
      DogLog.log("Arm/HighestAngle", highestSeenAngle);
    }
    if (rangeOfMotionGood()) {
      DogLog.clearFault("Arm not seen range of motion");
    } else {
      DogLog.logFault("Arm not seen range of motion", AlertType.kWarning);
    }

    switch (getState()) {
      case PRE_MATCH_HOMING -> {
        if (rangeOfMotionGood()) {
          if (DriverStation.isEnabled()) {
            motor.setPosition(
                Units.degreesToRotations(
                    RobotConfig.get().arm().minAngle() + (motorAngle - lowestSeenAngle)));

            setStateFromRequest(ArmState.CORAL_STOWED);
          } else {
            motor.setControl(brakeNeutralRequest);
          }
        }
      }
      case COLLISION_AVOIDANCE -> {
        motor.setControl(
            pidRequest.withPosition(Units.degreesToRotations(clamp(collisionAvoidanceGoal))));
      }
      default -> {}
    }
  }

  @Override
  protected ArmState getNextState(ArmState currentState) {
    if (currentState == ArmState.MID_MATCH_HOMING
        && averageMotorCurrent > RobotConfig.get().arm().homingCurrentThreshold()) {
      motor.setPosition(Units.degreesToRotations(RobotConfig.get().arm().homingPosition()));
      return ArmState.CORAL_STOWED;
    }

    // Don't do anything
    return currentState;
  }

  public boolean rangeOfMotionGood() {
    return (highestSeenAngle - lowestSeenAngle) > MINIMUM_EXPECTED_HOMING_ANGLE_CHANGE;
  }

  private static double clamp(double armAngle) {
    return MathUtil.clamp(
        armAngle, RobotConfig.get().arm().minAngle(), RobotConfig.get().arm().maxAngle());
  }
}
