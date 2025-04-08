package frc.robot.autos;

import frc.robot.autos.auto_path_commands.blue.BlueDoNothingAuto;
import frc.robot.autos.auto_path_commands.red.RedDoNothingAuto;
import frc.robot.autos.auto_path_commands.red.RedLollipopAuto;
import frc.robot.autos.auto_path_commands.red.RedLollipopVariantAuto;
import frc.robot.autos.auto_path_commands.red.RedPushPartnerAuto;
import frc.robot.autos.auto_path_commands.red.RedThreePiece2ILKAuto;
import frc.robot.autos.auto_path_commands.red.RedThreePiece3IKLAuto;
import frc.robot.robot_manager.RobotManager;
import java.util.function.BiFunction;

public enum AutoSelection {
  DO_NOTHING(RedDoNothingAuto::new, BlueDoNothingAuto::new),
  THREE_PIECE_2ILK(RedThreePiece2ILKAuto::new, RedThreePiece2ILKAuto::new),
  THREE_PIECE_3IKL(RedThreePiece3IKLAuto::new, RedThreePiece3IKLAuto::new),
  LOLLIPOP(RedLollipopAuto::new, RedLollipopAuto::new),
  LOLLIPOP_VARIANT(RedLollipopVariantAuto::new, RedLollipopVariantAuto::new),
  PUSH_PARTNER(RedPushPartnerAuto::new, RedPushPartnerAuto::new);

  public final BiFunction<RobotManager, Trailblazer, BaseAuto> redAuto;
  public final BiFunction<RobotManager, Trailblazer, BaseAuto> blueAuto;

  private AutoSelection(
      BiFunction<RobotManager, Trailblazer, BaseAuto> redAuto,
      BiFunction<RobotManager, Trailblazer, BaseAuto> blueAuto) {
    this.redAuto = redAuto;
    this.blueAuto = blueAuto;
  }
}
