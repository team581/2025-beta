package frc.robot.util.scheduling;

public enum SubsystemPriority {
  // 20-29 is for sensor subsystems

  // IMU runs before vision so that it has fresh data to pass to MegaTag2
  IMU(23),
  // Vision runs before localization so that it has fresh vision data for pose estimator
  VISION(22),
  // Swerve needs to have fresh data for localization
  SWERVE(21),
  // Localization runs before arm and shooter so that they have fresh speaker distance values
  LOCALIZATION(20),
  FMS(20),

  // 10-19 is for actuator subsystems
  // Intake must run before pivot so that it has fresh sensor data
  INTAKE(11),
  PIVOT(10),
  ELEVATOR(10),
  WRIST(10),
  LIGHTS(10),

  // 0-9 is for manager subsystems
  ROBOT_MANAGER(9),

  // Robot manager runs last so that all sensor data is fresh before processing state transitions
  AUTOS(0);

  final int value;

  private SubsystemPriority(int priority) {
    this.value = priority;
  }

  // TODO: Tweak subsystem priority values when more of the subsystems are added
}
