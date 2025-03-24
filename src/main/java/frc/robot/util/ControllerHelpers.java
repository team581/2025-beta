package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;

public final class ControllerHelpers {
  public static double deadbandJoystickValue(double joystickValue, double deadband) {
    return MathUtil.applyDeadband(joystickValue, deadband, 1);
  }

  public static Translation2d fromCircularDiscCoordinates(double x, double y) {
    // https://stackoverflow.com/a/32391780
    var mappedX =
        0.5 * MathHelpers.signedSqrt(2 + Math.pow(x, 2) - Math.pow(y, 2) + 2 * x * Math.sqrt(2))
            - 0.5
                * MathHelpers.signedSqrt(
                    2 + Math.pow(x, 2) - Math.pow(y, 2) - 2 * x * Math.sqrt(2));
    var mappedY =
        0.5 * MathHelpers.signedSqrt(2 - Math.pow(x, 2) + Math.pow(y, 2) + 2 * y * Math.sqrt(2))
            - 0.5
                * MathHelpers.signedSqrt(
                    2 - Math.pow(x, 2) + Math.pow(y, 2) - 2 * y * Math.sqrt(2));

    return desaturate(mappedX, mappedY);
  }

  private static Translation2d desaturate(double x, double y) {
    double absX = Math.abs(x);
    double absY = Math.abs(y);

    if (absX > 1 && absX > absY) {
      // X is too big and is the more problematic one

      var ratio = 1 / absX;

      return new Translation2d(x * ratio, y * ratio);
    } else if (absY > 1 && absY > absX) {
      // Y is too big and is the more problematic one
      var ratio = 1 / absY;

      return new Translation2d(x * ratio, y * ratio);

    } else {
      //  Everything fine
      return new Translation2d(x, y);
    }
  }

  private ControllerHelpers() {}
}
