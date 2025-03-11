// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// ill do this later
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

import com.ctre.phoenix6.hardware.TalonFX;

public class climber extends SubsystemBase {
  /** Creates a new climber. */
 TalonFX climbb;
  public climber() {
    climbb = new TalonFX(25);
  }
  public void rise(){
    climbb.set(10);
  }
  public void descend(){
    climbb.set(-10);
  }
  public void stopClimb(){
    climbb.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
