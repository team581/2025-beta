package frc.robot.intake_deploy;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class DeploySubsystem extends StateMachine<DeployState> {
  private final double TOLERANCE = 1.0;

  private final TalonFX motor;
  private final MotionMagicVoltage positionRequest =
      new MotionMagicVoltage(Units.degreesToRotations(DeployState.UNHOMED.angle));
  private final LinearFilter currentFilter = LinearFilter.movingAverage(7);

  private double currentAngle = 0.0;
  private double rawCurrent = 0.0;
  private double filteredCurrent = 0.0;

  public DeploySubsystem(TalonFX motor) {
    super(SubsystemPriority.DEPLOY, DeployState.UNHOMED);

    motor.getConfigurator().apply(RobotConfig.get().deploy().motorConfig());

    this.motor = motor;
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
          motor.setControl(positionRequest.withPosition(Units.degreesToRotations(newState.angle)));
    }
  }

  @Override
  protected DeployState getNextState(DeployState currentState) {
    return switch (currentState) {
      case HOMING -> {
        if (filteredCurrent > RobotConfig.get().deploy().homingCurrentThreshold()) {
          motor.setPosition(Units.degreesToRotations(currentState.angle));
          yield DeployState.STOWED;
        }
        yield currentState;
      }
      default -> currentState;
    };
  }

  @Override
  public void robotPeriodic() {
      super.robotPeriodic();

      DogLog.log("Deploy/AtGoal", atGoal());
      DogLog.log("Deploy/Angle", currentAngle);
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
      default -> MathUtil.isNear(getState().angle, currentAngle, TOLERANCE);
    };
  }

  public void setState(DeployState newState) {
    setStateFromRequest(newState);
  }
}
