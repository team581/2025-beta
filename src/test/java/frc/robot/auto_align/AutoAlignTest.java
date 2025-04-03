package frc.robot.auto_align;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

public class AutoAlignTest {

  @Test
  void scoreRightOrLeft() {
    var robotPose = new Pose2d(13, 0, Rotation2d.fromDegrees(180.0));
    var result = AutoAlign.getScoringSideFromRobotPose(robotPose, true, true);
    assertEquals(RobotScoringSide.RIGHT, result);
  }
}
