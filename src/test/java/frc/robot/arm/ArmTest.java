package frc.robot.arm;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class ArmTest {
  @Test
  void test1() {
    var currentAngle = 90.0;
    var normalizedGoalAngle = 90.0;

    var result = ArmSubsystem.denormalizeAngleForward(currentAngle, normalizedGoalAngle);

    assertEquals(90, result);
  }

  @Test
  void test2() {
    var currentAngle = 90.0;
    var normalizedGoalAngle = 90.0;

    var result = ArmSubsystem.denormalizeAngleBackward(currentAngle, normalizedGoalAngle);

    assertEquals(90, result);
  }

  @Test
  void test3() {
    var currentAngle = 0;
    var normalizedGoalAngle = -90;

    var result = ArmSubsystem.denormalizeAngleForward(currentAngle, normalizedGoalAngle);

    assertEquals(270, result);
  }

  @Test
  void test4() {
    var currentAngle = 0;
    var normalizedGoalAngle = 90;

    var result = ArmSubsystem.denormalizeAngleBackward(currentAngle, normalizedGoalAngle);

    assertEquals(-270, result);
  }

  @Test
  void test5() {
    var currentAngle = 360;
    var normalizedGoalAngle = 180;

    var result = ArmSubsystem.denormalizeAngleForward(currentAngle, normalizedGoalAngle);

    assertEquals(540, result);
  }

  @Test
  void test6() {
    var currentAngle = 360;
    var normalizedGoalAngle = 180;

    var result = ArmSubsystem.denormalizeAngleBackward(currentAngle, normalizedGoalAngle);

    assertEquals(180, result);
  }
}
