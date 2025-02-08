// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.math.controller.PIDController;


public class cArm extends SubsystemBase {
  /** Creates a new cArm. */
  TalonFX coralArm;
  PIDController cArmPID ;
  final PositionVoltage request;
  private double aim;
  private Encoder cArmEncoder;
  public cArm() {
    coralArm = new TalonFX(OperatorConstants.cArmId);
    cArmEncoder = new Encoder(0,1);

    var slot0Configs = new Slot0Configs();
    // slot0Configs.kV = 0.12;
      slot0Configs.kP = 2;
      slot0Configs.kI = 0;
      slot0Configs.kD = 0;
      coralArm.getConfigurator().apply(slot0Configs);
       request = new PositionVoltage(0).withSlot(0);
 
  
  }
  public void turnPoint(){
    // System.out.println(elMotor.getRotorPosition().getValue());
    if(Math.abs(aim-coralArm.getRotorPosition().getValueAsDouble())>1){
      coralArm.setControl(request.withPosition(aim));
    }
    else{
      coralArm.set(0);
    }
    

  }
  public void setcArm(double goal){
    aim= goal;
  }

  public void stopcArm(){
    coralArm.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
}
