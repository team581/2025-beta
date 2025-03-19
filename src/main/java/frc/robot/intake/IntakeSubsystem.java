package frc.robot.intake;

import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.S2StateValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.config.FeatureFlags;
import frc.robot.config.RobotConfig;
import frc.robot.util.VelocityDetector;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class IntakeSubsystem extends StateMachine<IntakeState> {
  private final TalonFX motor;
  private final CANdi candi;
  private final Debouncer debouncer = RobotConfig.get().intake().debouncer();

  private double motorVelocity = 0.0;
  private boolean CoralVelocityGp = false;
  private boolean sensorsHaveGP = false;
  private boolean sensorRaw = false;

  private boolean sensorDebounced = false;

  private final VelocityDetector CoralDetection = new VelocityDetector(78, 0.2, 0.1);

  public IntakeSubsystem(TalonFX motor, CANdi candi) {
    super(SubsystemPriority.INTAKE, IntakeState.IDLE_NO_GP);

    motor.getConfigurator().apply(RobotConfig.get().claw().motorConfig());
    this.motor = motor;
    this.candi = candi;
  }

  @Override
  protected void collectInputs() {
    CoralVelocityGp = CoralDetection.hasGamePiece(motorVelocity, 65);

    double motorVelocity = motor.getVelocity().getValueAsDouble();
    DogLog.log("Intake/MotorVelocity", motorVelocity);
    sensorRaw = candi.getS2State().getValue() != S2StateValue.Low;
    sensorDebounced = debouncer.calculate(sensorRaw);
  }

  public boolean getHasGP() {
    return switch (getState()) {
      case INTAKING ->
          FeatureFlags.INTAKE_VELOCITY_CORAL_DETECTION.getAsBoolean()
              ? CoralVelocityGp && sensorsHaveGP
              : sensorsHaveGP;
      default -> sensorsHaveGP;
    };
  }

  public void setState(IntakeState newState) {
    setStateFromRequest(newState);
  }

  @Override
  protected void afterTransition(IntakeState newState) {
    switch (newState) {
      case IDLE_NO_GP -> {
        motor.disable();
      }
      case IDLE_GP -> {
        motor.setVoltage(0);
      }
      case INTAKING -> {
        motor.setVoltage(0);
        CoralDetection.reset();
      }
      case SCORING -> {
        motor.setVoltage(0);
      }
      case OUTTAKING -> {
        motor.setVoltage(0);
      }
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    DogLog.log("Intake/AppliedVoltage", motor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Intake/Sensors/RightSensorDebounced", sensorDebounced);
    DogLog.log("Intake/SensorsHaveGP", sensorsHaveGP);
  }
}
