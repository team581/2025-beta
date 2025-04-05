package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

public class ControllerHelpersTest {
  @Test
  void circularDiscCoordinateTest() {
    var result = ControllerHelpers.fromCircularDiscCoordinates(-0.68, -0.58);
    var expected = new Translation2d(-0.78, -0.7);

    assertEquals(expected, MathHelpers.roundTo(result, 2));
  }

  @Test
  void deadbandJoystickValueTest() {
    var result = MathHelpers.roundTo(ControllerHelpers.deadbandJoystickValue(0.5, 0.05), 3);

    assertEquals(result, 0.474);
  }
}
