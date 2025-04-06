package frc.robot.robot_manager;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.arm.ArmState;
import frc.robot.arm.ArmSubsystem;
import frc.robot.config.RobotConfig;
import frc.robot.elevator.ElevatorState;
import frc.robot.util.MathHelpers;
import java.util.Objects;

public record SuperstructurePosition(
    double elevatorHeight,
    double armAngle,
    TrapezoidProfile.State elevatorState,
    TrapezoidProfile.State armState) {
  // Elevator heights are accurate to 0.1 inches
  private static final double ELEVATOR_PRECISION = 0.1;
  // Arm angles are accurate to 0.1 degrees
  private static final double ARM_PRECISION = 0.1;
  private static final TrapezoidProfile ELEVATOR_PROFILE =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              RobotConfig.get().elevator().leftMotorConfig().MotionMagic.MotionMagicCruiseVelocity,
              RobotConfig.get().elevator().leftMotorConfig().MotionMagic.MotionMagicAcceleration));
  private static final TrapezoidProfile ARM_PROFILE =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              Units.rotationsToDegrees(
                  RobotConfig.get().arm().motorConfig().MotionMagic.MotionMagicCruiseVelocity),
              Units.rotationsToDegrees(
                  RobotConfig.get().arm().motorConfig().MotionMagic.MotionMagicAcceleration)));

  // TODO: Tune this number, maybe the big tolerances in collision avoidance mean we can reduce it?
  private static final double STATIC_COST = 1;

  public SuperstructurePosition(double elevatorHeight, double armAngle) {
    this(
        elevatorHeight,
        armAngle,
        new TrapezoidProfile.State(elevatorHeight, 0),
        new TrapezoidProfile.State(armAngle, 0));
  }

  public SuperstructurePosition(ElevatorState elevatorState, double armAngle) {
    this(elevatorState.getHeight(), armAngle);
  }

  public SuperstructurePosition(double elevatorHeight, ArmState armState) {
    this(elevatorHeight, armState.getAngle());
  }

  public SuperstructurePosition(ElevatorState elevatorState, ArmState armState) {
    this(elevatorState.getHeight(), armState.getAngle());
  }

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
      return isNear(otherPosition, ELEVATOR_PRECISION, ARM_PRECISION);
    }

    return false;
  }

  public boolean isNear(
      SuperstructurePosition other, double elevatorTolerance, double armTolerance) {
    return MathUtil.isNear(elevatorHeight, other.elevatorHeight, elevatorTolerance)
        && MathUtil.isNear(armAngle, other.armAngle, armTolerance, -180, 180);
  }

  /**
   * The cost of moving from this position to another position.
   *
   * @param other The position you are going to.
   */
  public double costFor(SuperstructurePosition other) {
    ELEVATOR_PROFILE.calculate(0.02, elevatorState, other.elevatorState);
    ARM_PROFILE.calculate(0.02, armState, other.armState);

    return ELEVATOR_PROFILE.totalTime() + ARM_PROFILE.totalTime() + STATIC_COST;
  }

  public double costForLongWay(SuperstructurePosition other) {
    ELEVATOR_PROFILE.calculate(0.02, elevatorState, other.elevatorState);
    ARM_PROFILE.calculate(
        0.02,
        armState,
        new TrapezoidProfile.State((360 - Math.abs(this.armAngle - other.armAngle)), 0));

    return ELEVATOR_PROFILE.totalTime() + ARM_PROFILE.totalTime() + STATIC_COST;
  }

  public Translation2d getTranslation() {
    return new Translation2d(0, Units.inchesToMeters(elevatorHeight))
        .plus(new Translation2d(ArmSubsystem.ARM_LENGTH_METERS, Rotation2d.fromDegrees(armAngle)));
  }
}
