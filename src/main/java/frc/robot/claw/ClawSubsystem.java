package frc.robot.claw;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.S1StateValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.config.FeatureFlags;
import frc.robot.config.RobotConfig;
import frc.robot.util.VelocityDetector;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class ClawSubsystem extends StateMachine<ClawState> {
  private final TalonFX motor;
  private final CANdi candi;
  private final Debouncer debouncer = RobotConfig.get().claw().debouncer();

  private final TorqueCurrentFOC algaeHoldRequest =
      new TorqueCurrentFOC(RobotConfig.get().claw().algaeHoldCurrent())
          .withMaxAbsDutyCycle(RobotConfig.get().claw().algaeHoldMaxDutyCycle());

  private boolean sensorRaw = false;
  private boolean sensorDebounced = false;
  private boolean sensorsHaveGP = false;

  private double motorVelocity = 0.0;

  private final VelocityDetector motorAlgaeDetection = new VelocityDetector(32, 0.2, 0.0);
  private boolean motorAlgaeVelocityGp = false;

  private final VelocityDetector motorCoralDetection = new VelocityDetector(78, 0.2, 0.1);
  private boolean motorCoralVelocityGp = false;

  public ClawSubsystem(TalonFX motor, CANdi candi) {
    super(SubsystemPriority.INTAKE, ClawState.IDLE_NO_GP);

    motor.getConfigurator().apply(RobotConfig.get().claw().motorConfig());
    this.motor = motor;
    this.candi = candi;
  }

  @Override
  protected void collectInputs() {
    motorVelocity = motor.getVelocity().getValueAsDouble();

    motorAlgaeVelocityGp = motorAlgaeDetection.hasGamePiece(motorVelocity, 20);

    motorCoralVelocityGp = motorCoralDetection.hasGamePiece(motorVelocity, 65);
    DogLog.log("Intake/Motor/Velocity", motorVelocity);

    sensorRaw = candi.getS1State().getValue() != S1StateValue.Low;

    sensorDebounced = debouncer.calculate(sensorRaw);

    sensorsHaveGP = sensorDebounced;
  }

  public boolean getSensor() {
    return sensorDebounced;
  }

  public boolean getHasGP() {
    return switch (getState()) {
      case CORAL_HANDOFF ->
          FeatureFlags.INTAKE_VELOCITY_CORAL_DETECTION.getAsBoolean()
              ? motorCoralVelocityGp && sensorsHaveGP
              : sensorsHaveGP;
      case INTAKING_ALGAE -> motorAlgaeVelocityGp;
      default -> sensorsHaveGP;
    };
  }

  public void setState(ClawState newState) {
    setStateFromRequest(newState);
  }

  @Override
  protected void afterTransition(ClawState newState) {
    switch (newState) {
      case IDLE_NO_GP -> {
        motor.disable();
      }
      case IDLE_W_ALGAE -> {
        motor.setVoltage(0.0);
      }
      case IDLE_W_CORAL -> {
        motor.setVoltage(0.0);
      }
      case INTAKING_ALGAE -> {
        motor.setVoltage(0.0);
        motorAlgaeDetection.reset();
      }
      case CORAL_HANDOFF -> {
        motor.setVoltage(0.0);
        motorCoralDetection.reset();
      }
      case SCORE_ALGAE_NET -> {
        motor.setVoltage(0.0);
      }
      case SCORE_ALGAE_PROCESSOR -> {
        motor.setVoltage(0.0);
      }
      case SCORE_CORAL -> {
        motor.setVoltage(0.0);
      }
      case OUTTAKING -> {
        motor.setVoltage(0.0);
      }
      case TRUFFLA_CORAL_INTAKE -> {
        motor.setVoltage(0.0);
      }
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    DogLog.log("Claw/Motor/AppliedVoltage", motor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Claw/Sensors/SensorRaw", sensorRaw);
    DogLog.log("Claw/Sensors/SensorDebounced", sensorDebounced);
    DogLog.log("Claw/SensorsHaveGP", sensorsHaveGP);
  }
}
