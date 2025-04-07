package frc.robot.intake_deploy;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.util.tuning.TunablePid;

public class DeploySubsystem extends StateMachine<DeployState> {
  private static final double TOLERANCE = 1.0;

  private final TalonFX motor;
  private final PositionVoltage positionRequest = new PositionVoltage(0).withEnableFOC(false);
  private final LinearFilter currentFilter = LinearFilter.movingAverage(7);

  private double currentAngle = 0.0;

  private double rawCurrent = 0.0;
  private double filteredCurrent = 0.0;

  private boolean clawHasGp = false;

  public DeploySubsystem(TalonFX motor) {
    super(SubsystemPriority.DEPLOY, DeployState.UNHOMED);

    motor.getConfigurator().apply(RobotConfig.get().deploy().motorConfig());

    this.motor = motor;
    TunablePid.of("Deploy", motor, RobotConfig.get().deploy().motorConfig());
  }

  @Override
  protected void afterTransition(DeployState newState) {
    switch (newState) {
      case UNHOMED -> {
        motor.disable();
      }
      case HOMING -> {
        motor.setVoltage(RobotConfig.get().deploy().homingVoltage());
      }
      default ->
          motor.setControl(
              positionRequest.withPosition(Units.degreesToRotations(newState.getAngle())));
    }
  }

  @Override
  protected DeployState getNextState(DeployState currentState) {
    return switch (currentState) {
      case HOMING -> {
        if (filteredCurrent > RobotConfig.get().deploy().homingCurrentThreshold()) {
          motor.setPosition(
              Units.degreesToRotations(RobotConfig.get().deploy().homingEndPosition()));
          yield DeployState.STOWED;
        }
        yield currentState;
      }
      case HANDOFF -> clawHasGp ? DeployState.STOWED : currentState;
      default -> currentState;
    };
  }

  public void setClawHasGP(boolean hasGP) {
    clawHasGp = hasGP;
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    DogLog.log("Deploy/AtGoal", atGoal());
    DogLog.log("Deploy/Angle", currentAngle);
    DogLog.log("Deploy/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Deploy/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Deploy/CurrentThreshold", RobotConfig.get().deploy().homingCurrentThreshold());
  }

  @Override
  protected void collectInputs() {
    rawCurrent = motor.getStatorCurrent().getValueAsDouble();
    filteredCurrent = currentFilter.calculate(rawCurrent);

    currentAngle = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
  }

  public boolean atGoal() {
    return switch (getState()) {
      case UNHOMED, HOMING -> false;
      default -> MathUtil.isNear(clamp(getState().getAngle()), currentAngle, TOLERANCE);
    };
  }

  public boolean atGoal(DeployState state) {
    return switch (getState()) {
      case UNHOMED, HOMING -> false;
      default -> MathUtil.isNear(clamp(state.getAngle()), currentAngle, TOLERANCE);
    };
  }

  public void setState(DeployState newState) {
    switch (getState()) {
      case HOMING -> {}
      case UNHOMED -> {
        if (newState == DeployState.HOMING) {
          setStateFromRequest(DeployState.HOMING);
        }
      }
      default -> {
        setStateFromRequest(newState);
      }
    }
  }

  private static double clamp(double deployAngle) {
    return MathUtil.clamp(
        deployAngle, RobotConfig.get().deploy().minAngle(), RobotConfig.get().deploy().maxAngle());
  }

  public double getAngle() {
    return currentAngle;
  }

  private final TalonFXConfiguration simMotorConfig = new TalonFXConfiguration();
  private TrapezoidProfile.Constraints simConstraints;
  private boolean simDidInit = false;

  @Override
  public void simulationPeriodic() {
    if (getState() == DeployState.HOMING) {
      motor.setPosition(RobotConfig.get().deploy().homingEndPosition());
      setStateFromRequest(DeployState.STOWED);
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
