package frc.robot;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.config.RobotConfig;

public class Hardware {
  // public final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

  public final TalonFX deployMotor =
      new TalonFX(RobotConfig.get().deploy().motorId(), RobotConfig.get().deploy().canBusName());

  public final TalonFX elevatorLeftMotor =
      new TalonFX(
          RobotConfig.get().elevator().leftMotorId(), RobotConfig.get().elevator().canBusName());
  public final TalonFX elevatorRightMotor =
      new TalonFX(
          RobotConfig.get().elevator().rightMotorId(), RobotConfig.get().elevator().canBusName());

  public final CommandXboxController driverController = new CommandXboxController(0);
  public final CommandXboxController operatorController = new CommandXboxController(1);

  public final TalonFX intakeMotor =
      new TalonFX(RobotConfig.get().intake().motorId(), RobotConfig.get().intake().canBusName());

  public final TalonFX clawMotor = new TalonFX(RobotConfig.get().claw().motorId());

  public final CANdi clawCandi =
      new CANdi(RobotConfig.get().claw().candiId(), RobotConfig.get().claw().canBusName());

  public final CANdi intakeCandi =
      new CANdi(RobotConfig.get().intake().candiId(), RobotConfig.get().intake().canBusName());

  public final TalonFX armMotor =
      new TalonFX(RobotConfig.get().arm().motorId(), RobotConfig.get().arm().canBusName());

  public final CANdle candle =
      new CANdle(RobotConfig.get().lights().candleId(), RobotConfig.get().lights().canBusName());

  public final TalonFX climberClimbMotor =
      new TalonFX(
          RobotConfig.get().climber().climbMotorId(), RobotConfig.get().climber().canBusName());

  public final CANcoder climberCANcoder =
      new CANcoder(
          RobotConfig.get().climber().cancoderId(), RobotConfig.get().climber().canBusName());

  public final TalonFX climberGrabMotor =
      new TalonFX(
          RobotConfig.get().climber().grabMotorId(), RobotConfig.get().climber().canBusName());

  public final CANrange climberCanrange =
      new CANrange(
          RobotConfig.get().climber().canrangeId(), RobotConfig.get().climber().canBusName());
}
