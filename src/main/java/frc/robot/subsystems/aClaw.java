// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;

public class aClaw extends SubsystemBase {
private final Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  // private DoubleSolenoid aDobSol = new DoubleSolenoid(0,PneumaticsModuleType.CTREPCM,OperatorConstants.fDoubSolA, OperatorConstants.rDoubSolA);
  private DoubleSolenoid aDobSol = new DoubleSolenoid(0,PneumaticsModuleType.CTREPCM,2, 3);

  Boolean close= true
  ;
 
  /** Creates a new aClaw. */
  public aClaw() {
    compressor.enableDigital();
  }
  public void closeClawA(){
    if(close){
      aDobSol.set(DoubleSolenoid.Value.kForward);

    }
    else{
      aDobSol.set(DoubleSolenoid.Value.kReverse);
     
    }
    close= !close;

  }
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}