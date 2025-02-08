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
 private final Compressor compressor = new Compressor(1, PneumaticsModuleType.CTREPCM);
  private DoubleSolenoid dobSolLeft = new DoubleSolenoid(1,PneumaticsModuleType.CTREPCM,OperatorConstants.fDoubSolA1, OperatorConstants.rDoubSolA1);
  private DoubleSolenoid dobSolRight = new DoubleSolenoid(1,PneumaticsModuleType.CTREPCM,OperatorConstants.fDoubSolA2, OperatorConstants.rDoubSolA2);
  Boolean closeLeft= true;
  Boolean closeRight= true;//same
  /** Creates a new aClaw. */
  public aClaw() {
    compressor.enableDigital();
  }
  public void closeClawA(){
    if(closeLeft&&closeRight){
      dobSolLeft.set(DoubleSolenoid.Value.kForward);
      dobSolRight.set(DoubleSolenoid.Value.kForward);
    }
    else{
      dobSolLeft.set(DoubleSolenoid.Value.kReverse);
      dobSolRight.set(DoubleSolenoid.Value.kReverse);
    }
    closeLeft= !closeLeft;
    closeRight= !closeRight;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}