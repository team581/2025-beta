package frc.robot.util.state_machines;

import java.util.function.Supplier;

public class PeriodicInput<T> implements Supplier<T> {
  private final Supplier<T> supplier;
  private T lastValue;

  public PeriodicInput(Supplier<T> supplier, T initialValue) {
    this.supplier = supplier;
    this.lastValue = initialValue;
  }

  @Override
  public T get() {
    return lastValue;
  }

  public void calculate() {
    lastValue = supplier.get();
  }
}
