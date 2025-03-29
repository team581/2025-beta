package frc.robot.arm;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
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

  public void setCollisionAvoidanceGoal(double angle) {
    collisionAvoidanceGoal = angle;
    DogLog.log("Arm/CollisionAvoidanceGoalAngle", collisionAvoidanceGoal);
  }

  public boolean atGoal() {
    return switch (getState()) {
      default -> MathUtil.isNear(getState().getAngle(), motorAngle, 2, -180, 180);
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
    DogLog.log("Arm/NormalizedAngle", motorAngle);
    DogLog.log("Arm/RawAngle", rawMotorAngle);
    DogLog.log("Arm/AtGoal", atGoal());
    DogLog.log("Arm/RangeOfMotionGood", rangeOfMotionGood());
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
      case SPIN_TO_WIN -> {
        motor.setControl(spinToWin);
      }
      default -> {}
    }
  }

  public boolean rangeOfMotionGood() {
    return Math.abs(highestSeenAngle - lowestSeenAngle) > MINIMUM_EXPECTED_HOMING_ANGLE_CHANGE;
  }
}
