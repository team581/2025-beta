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
      new TalonFX(RobotConfig.get().deploy().motorID(), RobotConfig.get().deploy().canBusName());

  public final TalonFX elevatorLeftMotor =
      new TalonFX(
          RobotConfig.get().elevator().leftMotorID(), RobotConfig.get().elevator().canBusName());
  public final TalonFX elevatorRightMotor =
      new TalonFX(
          RobotConfig.get().elevator().rightMotorID(), RobotConfig.get().elevator().canBusName());

  public final CommandXboxController driverController = new CommandXboxController(0);
  public final CommandXboxController operatorController = new CommandXboxController(1);

  public final TalonFX intakeTopMotor =
      new TalonFX(RobotConfig.get().intake().topMotorID(), RobotConfig.get().intake().canBusName());
  public final TalonFX intakeBottomMotor =
      new TalonFX(
          RobotConfig.get().intake().bottomMotorID(), RobotConfig.get().intake().canBusName());
  public final CANdi candi =
      new CANdi(RobotConfig.get().intake().candiID(), RobotConfig.get().intake().canBusName());

  public final TalonFX wristMotor =
      new TalonFX(RobotConfig.get().wrist().motorID(), RobotConfig.get().wrist().canBusName());

  public final TalonFX rollMotor =
      new TalonFX(RobotConfig.get().roll().motorID(), RobotConfig.get().roll().canBusName());

  public final CANdle candle =
      new CANdle(RobotConfig.get().lights().candleID(), RobotConfig.get().lights().canBusName());

  public final TalonFX climberClimbMotor =
      new TalonFX(
          RobotConfig.get().climber().climbMotorID(), RobotConfig.get().climber().canBusName());

  public final CANcoder climberCANcoder =
      new CANcoder(
          RobotConfig.get().climber().cancoderID(), RobotConfig.get().climber().canBusName());

  public final TalonFX climberGrabMotor =
      new TalonFX(
          RobotConfig.get().climber().grabMotorID(), RobotConfig.get().climber().canBusName());

  public final CANrange climberCanrange =
      new CANrange(
          RobotConfig.get().climber().canrangeID(), RobotConfig.get().climber().canBusName());
}
