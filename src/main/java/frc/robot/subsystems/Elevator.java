// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  TalonFX elMotor;
  final DutyCycleOut dy;
  PIDController elPID ;
  public Elevator() {
    elMotor = new TalonFX(23);
    CurrentLimitsConfigs eleLimCon = new CurrentLimitsConfigs().withStatorCurrentLimitEnable(true).withStatorCurrentLimit(40).withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(40);
    elMotor.getConfigurator().apply(eleLimCon);
    elPID = new PIDController(5,0 ,0);
  //      var slot0Configs = new Slot0Configs();
  //  slot0Configs.kP = 24;
  //  slot0Configs.kI = 0;
  //  slot0Configs.kD = 0.1;
  //  elMotor.getConfigurator().apply(slot0Configs);
    // CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
    // cc_cfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    // cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    // cc_cfg.MagnetSensor.MagnetOffset = 0.4;
    // elMotor.getConfigurator().apply(cc_cfg);

    // TalonFXConfiguration fx_cfg = new TalonFXConfiguration();
    // fx_cfg.Feedback.FeedbackRemoteSensorID = elMotor.getDeviceID();
    // fx_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    // fx_cfg.Feedback.SensorToMechanismRatio = 1.0;
    // fx_cfg.Feedback.RotorToSensorRatio = 12.8;

    // elMotor.getConfigurator().apply(fx_cfg);
//     var fx_cfg = new TalonFXConfiguration();
// fx_cfg.Feedback.FeedbackRemoteSensorID = m_cancoder.getDeviceID();
// fx_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

// m_talonFX.getConfigurator().apply(fx_cfg);
  
   dy = new DutyCycleOut(0);
  // elMotor.setSelectedSensorPosition(0);
  }
  public void toFloor(){
    // elMotor.setControl(new PositionDutyCycle(1));//set some angle or a double
  }
  public void toL1(){
    // elMotor.setControl(new PositionDutyCycle(null));//set some angle or a double
    // elMotor.set(.3);
  //  elMotor.setControl(dy.withOutput(0.3));
  //  var slot0Configs = new Slot0Configs();
  //  slot0Configs.kP = 24;
  //  slot0Configs.kI = 0;
  //  slot0Configs.kD = 0.1;
  //  elMotor.getConfigurator().apply(slot0Configs);
  elMotor.set(1);
  Timer.delay(2);
  elMotor.set(0);
  //  var position = new PositionVoltage(0).withSlot(0);
  //  elMotor.setControl(position.withPosition(10));

  // elMotor.set(ControlMode.PercentOutput,elPID.calculate());

  }
  public void toL2(){
    elMotor.set(1);
    Timer.delay(3);
    elMotor.set(0);
  }
  public void toL3(){
    elMotor.set(1);
  Timer.delay(4);
  elMotor.set(0);
  }
  public void toL4(){
    elMotor.set(1);
    Timer.delay(5);
    elMotor.set(0);
  }
  public void stopElevator(){
    elMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
