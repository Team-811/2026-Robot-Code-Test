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
import edu.wpi.first.wpilibj.Encoder;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  TalonFX elMotor;
  // final DutyCycleOut dy;
  PIDController elPID ;
  // final PositionVoltage request;
  public Encoder encoder;
  double pullyDiameter;
  double gearRatio;
  final PositionDutyCycle request;
  public Elevator() {
    elMotor = new TalonFX(24);
    encoder = new Encoder(0,1);
    // CurrentLimitsConfigs eleLimCon = new CurrentLimitsConfigs().withStatorCurrentLimitEnable(true).withStatorCurrentLimit(40).withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(40);
    // elMotor.getConfigurator().apply(eleLimCon); //need
    // elPID = new PIDController(5,0 ,0);
    // elPID.setTolerance(0.5);
    gearRatio = 25;
    pullyDiameter = 1.5;
       var slot0Configs = new Slot0Configs();
    // slot0Configs.kV = 0.12;
   slot0Configs.kP = 2.4;
   slot0Configs.kI = 0;
   slot0Configs.kD = 0.1;
   elMotor.getConfigurator().apply(slot0Configs);
    // request = new PositionVoltage(0).withSlot(0);
   request = new PositionDutyCycle(0);

  //  dy = new DutyCycleOut(0);

  }
  public double getHeight(){
    double rotations = encoder.get();
    double inchPerRo = (Math.PI*pullyDiameter)/gearRatio;
    return rotations *inchPerRo;
  }
  public void setTargetHeight(double targetHeight){
    double currentHeight = getHeight();
    double toHeight = elPID.calculate(currentHeight,targetHeight);
    elMotor.set(toHeight);
  }
  public void toFloor(){
    // elMotor.setControl(new PositionDutyCycle(1));//set some angle or a double
    // elMotor.setControl(ControlMode.PercentOutput,elPID.calculate(0.01),0);
  }
  public void toL1(){

  //  elMotor.setControl(dy.withOutput(0.3));
  //  var position = new PositionVoltage(0).withSlot(0);
  //  elMotor.setControl(position.withPosition(10));
  elMotor.setControl(request.withPosition(1));
  // elMotor.set(ControlMode.PercentOutput,elPID.calculate(0),0);

  }
  public void toL2(){
    // elMotor.set(1);
    // Timer.delay(3);
    // elMotor.set(0);
    elMotor.setControl(request.withPosition(2));
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
