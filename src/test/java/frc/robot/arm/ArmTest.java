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
  @Test
  void getRawAngleFromNormalAngleTest() {
    var goalAngle = -90;
    var rawAngle = -360;

    var result = ArmSubsystem.getRawAngleFromNormalAngleTest(goalAngle, rawAngle);

    assertEquals(-450, result);
  }
  @Test
  void getRawAngleFromNormalAngleTest2() {
    var goalAngle = -90;
    var rawAngle = 360;

    var result = ArmSubsystem.getRawAngleFromNormalAngleTest(goalAngle, rawAngle);

    assertEquals(270, result);
  }
  @Test
  void getRawAngleFromNormalAngleTest3() {
    var goalAngle = -90;
    var rawAngle = -180;

    var result = ArmSubsystem.getRawAngleFromNormalAngleTest(goalAngle, rawAngle);

    assertEquals(-90, result);
  }
  @Test
  void getRawAngleFromNormalAngleTest4() {
    var goalAngle = -90;
    var rawAngle = -181;

    var result = ArmSubsystem.getRawAngleFromNormalAngleTest(goalAngle, rawAngle);

    assertEquals(-90, result);
  } @Test
  void getRawAngleFromNormalAngleTest5() {
    var goalAngle = -90;
    var rawAngle = -179;

    var result = ArmSubsystem.getRawAngleFromNormalAngleTest(goalAngle, rawAngle);

    assertEquals(-90, result);
  }
}
