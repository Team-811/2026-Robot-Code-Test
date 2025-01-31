// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.Compressor;
// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// public class cClaw extends SubsystemBase {
//   /** Creates a new claw. */
//   private final Compressor compressor = new Compressor(1, PneumaticsModuleType.CTREPCM);
//   // private final DoubleSolenoid dobSol1 = new DoubleSolenoid(Constants.OperatorConstants.CompressorID_C,PneumaticsModuleType.CTREPCM,Constants.OperatorConstants.SolenoidID_In_1C, Constants.OperatorConstants.SolenoidID_Out_1C);
//   Boolean closeClaw = true; // could be reversed 
//   // private final DoubleSolenoid dobSol2 = new DoubleSolenoid(Constants.OperatorConstants.CompressorID_C,PneumaticsModuleType.CTREPCM,Constants.OperatorConstants.SolenoidID_In_2C, Constants.OperatorConstants.SolenoidID_Out_2C);
//   Boolean turnClaw = true;//change if reversed
//   private final DoubleSolenoid dobSol1 = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 0, 1);
//   private final DoubleSolenoid dobSol2 = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 2, 3);
//   public cClaw() {
//     compressor.enableDigital();
//   }
//   public void solenoidToggle(){
//     if(closeClaw){
//       dobSol1.set(DoubleSolenoid.Value.kForward);
//     }
//     else{
//       dobSol1.set(DoubleSolenoid.Value.kReverse);
//     }
//     closeClaw = !closeClaw;
//   }
//   public void turn(){

//     if(turnClaw){
//       dobSol2.set(DoubleSolenoid.Value.kForward);
//     }
//     else{
//       dobSol2.set(DoubleSolenoid.Value.kReverse);
//     }
//     turnClaw = !turnClaw;

//     }


//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }
// }