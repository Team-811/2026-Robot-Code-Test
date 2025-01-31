// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.Compressor;
// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// public class aClaw extends SubsystemBase {
//  private final Compressor compressor = new Compressor(1, PneumaticsModuleType.CTREPCM);
//   private DoubleSolenoid dobSolLeft = new DoubleSolenoid(1,PneumaticsModuleType.CTREPCM,4, 5);
//   private DoubleSolenoid dobSolRight = new DoubleSolenoid(1,PneumaticsModuleType.CTREPCM,6, 7);
//   Boolean closeLeft= true;
//   //private DoubleSolenoid dobSolRight = new DoubleSolenoid(PneumaticsModuleType.REVPH,Constants.OperatorConstants.SolenoidID_In_2A, Constants.OperatorConstants.SolenoidID_Out_2A);
//   //private DoubleSolenoid dobSolLeft = new DoubleSolenoid(PneumaticsModuleType.REVPH,Constants.OperatorConstants.SolenoidID_In_1A, Constants.OperatorConstants.SolenoidID_Out_1A);
//   Boolean closeRight= true;//same
//   /** Creates a new aClaw. */
//   public aClaw() {
//     compressor.enableDigital();
//   }
//   public void closeClawA(){
//     if(closeLeft&&closeRight){
//       dobSolLeft.set(DoubleSolenoid.Value.kForward);
//       dobSolRight.set(DoubleSolenoid.Value.kForward);
//     }
//     else{
//       dobSolLeft.set(DoubleSolenoid.Value.kReverse);
//       dobSolRight.set(DoubleSolenoid.Value.kReverse);
//     }
//     closeLeft= !closeLeft;
//     closeRight= !closeRight;

//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }
// }