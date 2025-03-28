package frc.robot.robot_manager;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.MathHelpers;
import java.util.Objects;

import dev.doglog.DogLog;

public record SuperstructurePosition(double elevatorHeight, double armAngle) {
  private static final double ARM_LENGTH = Units.inchesToMeters(37.416);
  // Elevator heights are accurate to 0.1 inches
  private static final double ELEVATOR_PRECISION = 0.1;
  // Arm angles are accurate to 0.1 degrees
  private static final double ARM_PRECISION = 0.1;
  private static final double ARM_DEGREES_PER_SECOND =
      1.0 / 270.0; // TODO: Get more legit ratios for these
  private static final double ELEVATOR_INCHES_PER_SECOND = 1.0 / 20.0;
  private static final double STATIC_COST = 1.0; // UNTUNNED

  @Override
  public final int hashCode() {
    var roundedElevator = MathHelpers.roundTo(elevatorHeight, ELEVATOR_PRECISION);
    var roundedArm = MathHelpers.roundTo(armAngle, ARM_PRECISION);

    return Objects.hash(roundedElevator, roundedArm);
  }

  @Override
  public final boolean equals(Object other) {
    if (this == other) {
      return true;
    }

    if (other instanceof SuperstructurePosition otherPosition) {
      return near(
          otherPosition,
          new SuperstructurePosition(elevatorHeight, armAngle),
          ELEVATOR_PRECISION,
          ARM_PRECISION);
    }

    return false;
  }

  public static boolean near(
      SuperstructurePosition firstPosition,
      SuperstructurePosition secondPosition,
      double elevatorTolerance,
      double armTolerance) {
    return (MathUtil.isNear(
            firstPosition.elevatorHeight(), secondPosition.elevatorHeight(), elevatorTolerance)
        && MathUtil.isNear(
            firstPosition.armAngle(), secondPosition.armAngle(), armTolerance, -180, 180));
  }

  /**
   * The cost of moving from this position to another position.
   *
   * @param other The position you are going to.
   */
  public double costFor(SuperstructurePosition other) {
    return Math.abs(this.armAngle - other.armAngle) * ARM_DEGREES_PER_SECOND
        + Math.abs(this.elevatorHeight - other.elevatorHeight) * ELEVATOR_INCHES_PER_SECOND
        + STATIC_COST;
  }

  public Translation2d getTranslation() {
    return new Translation2d(0, Units.inchesToMeters(elevatorHeight))
        .plus(new Translation2d(ARM_LENGTH, Rotation2d.fromDegrees(armAngle)));
  }
}
