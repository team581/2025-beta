package frc.robot.autos;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.fms.FmsSubsystem;
import frc.robot.robot_manager.RobotManager;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class Autos extends LifecycleSubsystem {
  private final AutoChooser autoChooser;
  private final RobotManager robotManager;
  private final Trailblazer trailblazer;
  private boolean hasEnabledAuto = false;
  private Pair<AutoSelection, BaseAuto> selectedRed;
  private Pair<AutoSelection, BaseAuto> selectedBlue;
  private Command autoCommand;

  public Autos(RobotManager robotManager, Trailblazer trailblazer) {
    super(SubsystemPriority.AUTOS);

    autoChooser = new AutoChooser(robotManager, trailblazer);
    this.robotManager = robotManager;
    this.trailblazer = trailblazer;

    selectedRed =
        Pair.of(
            AutoSelection.DO_NOTHING,
            AutoSelection.DO_NOTHING.redAuto.apply(robotManager, trailblazer));
    selectedBlue =
        Pair.of(
            AutoSelection.DO_NOTHING,
            AutoSelection.DO_NOTHING.blueAuto.apply(robotManager, trailblazer));
    autoCommand = createAutoCommand();
  }

  public Command getAutoCommand() {
    return autoCommand;
  }

  @Override
  public void robotPeriodic() {
    if (DriverStation.isDisabled()) {
      updateSelection();

      if (!hasEnabledAuto) {
        // Continuously reset pose
        resetPoseForAuto();
      }
    }

    if (DriverStation.isAutonomousEnabled()) {
      hasEnabledAuto = true;
    }
  }

  private void resetPoseForAuto() {
    var auto = FmsSubsystem.isRedAlliance() ? selectedRed.getSecond() : selectedBlue.getSecond();
    var startingPose = auto.getStartingPose();
    robotManager.localization.resetPose(startingPose);
  }

  private void updateSelection() {
    // If anything about the auto selection has changed, fully recreate all the commands
    // This avoids potential errors from composed commands being used in multiple compositions
    if (selectedRed.getFirst() != autoChooser.getSelectedAuto()
        || selectedBlue.getFirst() != autoChooser.getSelectedAuto()) {
      selectedRed =
          Pair.of(
              autoChooser.getSelectedAuto(),
              autoChooser.getSelectedAuto().redAuto.apply(robotManager, trailblazer));

      selectedBlue =
          Pair.of(
              autoChooser.getSelectedAuto(),
              autoChooser.getSelectedAuto().blueAuto.apply(robotManager, trailblazer));

      autoCommand = createAutoCommand();
    }
  }

  private Command createAutoCommand() {
    return Commands.either(
            selectedRed.getSecond().getAutoCommand(),
            selectedBlue.getSecond().getAutoCommand(),
            FmsSubsystem::isRedAlliance)
        .withName(selectedRed.getSecond().name() + "_or_" + selectedBlue.getSecond().name());
  }
}
