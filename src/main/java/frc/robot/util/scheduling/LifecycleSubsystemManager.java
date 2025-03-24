package frc.robot.util.scheduling;

import com.google.common.collect.ImmutableSet;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Set;

public final class LifecycleSubsystemManager {
  @SuppressWarnings("unchecked")
  private static Set<Command> getScheduledCommands() {
    try {
      var field = CommandScheduler.class.getDeclaredField("m_scheduledCommands");
      field.setAccessible(true);
      var rawResult = field.get(CommandScheduler.getInstance());

      return (Set<Command>) rawResult;
    } catch (NoSuchFieldException | SecurityException | IllegalAccessException e) {
      DogLog.logFault("Failed to do reflection for scheduled commands");
      return Set.of();
    }
  }

  public static LifecycleStage getStage() {
    if (DriverStation.isTeleopEnabled()) {
      return LifecycleStage.TELEOP;
    } else if (DriverStation.isAutonomousEnabled()) {
      return LifecycleStage.AUTONOMOUS;
    } else if (DriverStation.isTestEnabled()) {
      return LifecycleStage.TEST;
    } else {

      return LifecycleStage.DISABLED;
    }
  }

  private static final List<LifecycleSubsystem> subsystems = new ArrayList<>();
  private static final CommandScheduler commandScheduler = CommandScheduler.getInstance();
  private static final ImmutableSet<Command> scheduledCommands =
      ImmutableSet.copyOf(getScheduledCommands());

  public static void ready() {
    subsystems.sort(
        Comparator.comparingInt((LifecycleSubsystem subsystem) -> subsystem.priority.value)
            .reversed());

    for (LifecycleSubsystem lifecycleSubsystem : subsystems) {
      commandScheduler.registerSubsystem(lifecycleSubsystem);
    }
  }

  public static void log() {
    DogLog.log(
        "Scheduler/ScheduledCommands",
        scheduledCommands.stream().map(command -> command.getName()).toArray(String[]::new));
  }

  static void registerSubsystem(LifecycleSubsystem subsystem) {
    subsystems.add(subsystem);
    commandScheduler.unregisterSubsystem(subsystem);
  }

  private LifecycleSubsystemManager() {}
}
