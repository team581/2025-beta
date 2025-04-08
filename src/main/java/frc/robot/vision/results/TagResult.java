package frc.robot.vision.results;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;

public record TagResult(Pose2d pose, double timestamp, Vector<N3> standardDevs) {}
