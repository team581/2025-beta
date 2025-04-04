package frc.robot;

import org.junit.jupiter.api.Test;

public class RobotTest {
  @Test
  void instantiateRobotTest() {
    var robot = new Robot();

    robot.robotInit();
    robot.robotPeriodic();

    robot.disabledInit();
    robot.disabledPeriodic();

    robot.autonomousInit();
    robot.autonomousPeriodic();

    robot.disabledInit();
    robot.disabledPeriodic();

    robot.teleopInit();
    robot.teleopPeriodic();

    robot.close();
  }
}
