package frc.robot.lights;

import com.ctre.phoenix.led.CANdle;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class LightsSubsystem extends StateMachine<LightsState> {
  private final CANdle candle;

  private final Timer blinkTimer = new Timer();
  private LightsState storedState = LightsState.IDLE_NO_GP;
  private LightsState disabledState = LightsState.HEALTHY;

  public LightsSubsystem(CANdle candle) {
    super(SubsystemPriority.LIGHTS, LightsState.IDLE_NO_GP);
    this.candle = candle;
    blinkTimer.start();
  }

  public void setState(LightsState newState) {
    setStateFromRequest(newState);
  }

  public void blink() {
    storedState = getState();
    setStateFromRequest(LightsState.BLINK);
  }

  public void setDisabledState(LightsState newDisabledState) {
    disabledState = newDisabledState;
  }

  @Override
  protected LightsState getNextState(LightsState currentState) {
    return switch (currentState) {
      case BLINK -> timeout(1.0) ? storedState : currentState;
      default -> currentState;
    };
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    var usedState = DriverStation.isDisabled() ? disabledState : getState();
    var color8Bit = new Color8Bit(usedState.color);
    if (usedState.pattern == BlinkPattern.SOLID) {
      candle.setLEDs(color8Bit.red, color8Bit.green, color8Bit.blue);
    } else {
      double time = blinkTimer.get();
      double onDuration = 0;
      double offDuration = 0;

      if (usedState.pattern == BlinkPattern.BLINK_FAST) {
        onDuration = BlinkPattern.BLINK_FAST.duration;
        offDuration = BlinkPattern.BLINK_FAST.duration * 2;
      } else if (usedState.pattern == BlinkPattern.BLINK_SLOW) {
        onDuration = BlinkPattern.BLINK_SLOW.duration;
        offDuration = BlinkPattern.BLINK_SLOW.duration * 2;
      }

      if (time >= offDuration) {
        blinkTimer.reset();
        candle.setLEDs(0, 0, 0);
      } else if (time >= onDuration) {
        candle.setLEDs(color8Bit.red, color8Bit.green, color8Bit.blue);
      }
    }

    DogLog.log("Lights/Color", usedState.color.toString());
    DogLog.log("Lights/Pattern", usedState.pattern);
  }
}
