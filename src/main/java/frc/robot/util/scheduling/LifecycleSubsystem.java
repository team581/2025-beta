package frc.robot.util.scheduling;

import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.Stopwatch;
import org.jspecify.annotations.Nullable;

/**
 * Extends {@link SubsystemBase} by adding in lifecycle methods for robotInit, teleopPeriodic, etc.,
 * similar to {@link Robot}.
 */
public class LifecycleSubsystem extends SubsystemBase {
  final SubsystemPriority priority;

  private final String loggerName;

  protected final String subsystemName;

  private @Nullable LifecycleStage previousStage = null;

  public LifecycleSubsystem(SubsystemPriority priority) {
    this.priority = priority;

    LifecycleSubsystemManager.registerSubsystem(this);

    String name = this.getClass().getSimpleName();
    name = name.substring(name.lastIndexOf('.') + 1);
    if (name.endsWith("Subsystem")) {
      name = name.substring(0, name.length() - "Subsystem".length());
    }
    subsystemName = name;
    loggerName = "Scheduler/LifecycleSubsystem/" + subsystemName + ".periodic()";
  }

  /** {@link IterativeRobotBase#robotPeriodic()} */
  public void robotPeriodic() {}

  /** {@link IterativeRobotBase#autonomousInit()} */
  public void autonomousInit() {}

  /** {@link IterativeRobotBase#autonomousPeriodic()} */
  public void autonomousPeriodic() {}

  /** {@link IterativeRobotBase#teleopInit()} */
  public void teleopInit() {}

  /** {@link IterativeRobotBase#teleopPeriodic()} */
  public void teleopPeriodic() {}

  /** {@link IterativeRobotBase#disabledInit()} */
  public void disabledInit() {}

  /** {@link IterativeRobotBase#disabledPeriodic()} */
  public void disabledPeriodic() {}

  @Override
  public void periodic() {
    Stopwatch.start(loggerName);

    LifecycleStage stage = LifecycleSubsystemManager.getStage();

    boolean isInit = previousStage != stage;

    robotPeriodic();

    switch (stage) {
      case DISABLED -> {
        if (isInit) {
          disabledInit();
        }

        disabledPeriodic();
      }
      case TELEOP -> {
        if (isInit) {
          teleopInit();
        }

        teleopPeriodic();
      }
      case AUTONOMOUS -> {
        if (isInit) {
          autonomousInit();
        }

        autonomousPeriodic();
      }
      case TEST -> {
        if (isInit) {
          testInit();
        }

        testPeriodic();
      }
    }

    Stopwatch.stop(loggerName);

    previousStage = stage;
  }

  /** {@link IterativeRobotBase#testInit()} */
  public void testInit() {}

  /** {@link IterativeRobotBase#testPeriodic()} */
  public void testPeriodic() {}
}
