package frc.robot.util;

import dev.doglog.DogLog;
import frc.robot.config.RobotConfig;
import java.util.function.BooleanSupplier;

public class FeatureFlag {
  public static BooleanSupplier of(String name, boolean defaultValue) {
    if (RobotConfig.IS_DEVELOPMENT) {
      return DogLog.tunable("FeatureFlags/" + name, defaultValue);
    }

    return () -> defaultValue;
  }

  private FeatureFlag() {}
}
