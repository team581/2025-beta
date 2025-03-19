package frc.robot.auto_align.field_calibration;

record Summary(MechanismState elevator, MechanismState arm, boolean align) {
  public String format() {
    var alignLabel = align ? "ok" : "bad";
    return "Elevator " + elevator.label + ", arm " + arm.label + ", align " + alignLabel;
  }

  public boolean isOk() {
    return elevator == MechanismState.OK && arm == MechanismState.OK && align;
  }
}
