package frc.robot.arm;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class ArmTest {
  @Test
  void getRawAngleFromNormalAngleTest() {
    var goalAngle = -90;
    var rawAngle = -360;

    var result = ArmSubsystem.getRawAngleFromNormalAngle(goalAngle, rawAngle);

    assertEquals(-450, result);
  }

  @Test
  void getRawAngleFromNormalAngleTest2() {
    var goalAngle = -90;
    var rawAngle = 360;

    var result = ArmSubsystem.getRawAngleFromNormalAngle(goalAngle, rawAngle);

    assertEquals(270, result);
  }

  @Test
  void getRawAngleFromNormalAngleTest3() {
    var goalAngle = -90;
    var rawAngle = -180;

    var result = ArmSubsystem.getRawAngleFromNormalAngle(goalAngle, rawAngle);

    assertEquals(-90, result);
  }

  @Test
  void getRawAngleFromNormalAngleTest4() {
    var goalAngle = -90;
    var rawAngle = -181;

    var result = ArmSubsystem.getRawAngleFromNormalAngle(goalAngle, rawAngle);

    assertEquals(-90, result);
  }

  @Test
  void getRawAngleFromNormalAngleTest5() {
    var goalAngle = -90;
    var rawAngle = -179;

    var result = ArmSubsystem.getRawAngleFromNormalAngle(goalAngle, rawAngle);

    assertEquals(-90, result);
  }
}
