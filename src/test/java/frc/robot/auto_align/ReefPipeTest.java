package frc.robot.auto_align;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.Test;

public class ReefPipeTest {
  private static final Translation2d BLUE_REEF_CENTER =
      new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));
  private static final Translation2d RED_REEF_CENTER =
      new Translation2d(Units.inchesToMeters(514.13), Units.inchesToMeters(158.499));

  @Test
  void checkBlueReefPipeDistancesToCenter() {
    var wantedDistance =
        ReefPipe.PIPE_A
            .getPose(ReefPipeLevel.BASE, false, RobotScoringSide.RIGHT)
            .getTranslation()
            .getDistance(BLUE_REEF_CENTER);

    for (var pipe : ReefPipe.values()) {
      var distance =
          pipe.getPose(ReefPipeLevel.BASE, false, RobotScoringSide.RIGHT)
              .getTranslation()
              .getDistance(BLUE_REEF_CENTER);
      assertTrue(MathUtil.isNear(wantedDistance, distance, 2));
    }
  }

  @Test
  void checkRedReefPipeDistancesToCenter() {
    var wantedDistance =
        ReefPipe.PIPE_A
            .getPose(ReefPipeLevel.BASE, true, RobotScoringSide.RIGHT)
            .getTranslation()
            .getDistance(RED_REEF_CENTER);

    for (var pipe : ReefPipe.values()) {
      var distance =
          pipe.getPose(ReefPipeLevel.BASE, true, RobotScoringSide.RIGHT)
              .getTranslation()
              .getDistance(RED_REEF_CENTER);
      // TODO: Figure out why a wider tolerance is needed here
      assertTrue(MathUtil.isNear(wantedDistance, distance, 0.02));
    }
  }
}
