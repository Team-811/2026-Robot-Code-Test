// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.
// // ill do this later
// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.OperatorConstants;

// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.controls.PositionVoltage;
// import com.ctre.phoenix6.hardware.TalonFX;

// public class climber extends SubsystemBase {
//   /** Creates a new climber. */
//  TalonFX climbb;
//  final PositionVoltage request;
  
//   private double point;
//   // DigitalInput limSwitch= new DigitalInput(0);
//   public climber() {
//     climbb = new TalonFX(25);
//     climbb.setPosition(0);
//       var slot0Configs = new Slot0Configs();
//       slot0Configs.kV = 0;
//       slot0Configs.kA = 0;
//      slot0Configs.kP = 0.2;
//      slot0Configs.kI = 0;
//      slot0Configs.kD = 0;
//      slot0Configs.kG =0;

//     climbb.getConfigurator().apply(slot0Configs);
//     request = new PositionVoltage(0).withSlot(0);


//   }
//   public void turnToPoint(){
//     // System.out.println(elMotor.getRotorPosition().getValue());
//     if(Math.abs(point-climbb.getRotorPosition().getValueAsDouble())>1){
//       climbb.setControl(request.withPosition(point));
//     }
//     else{
//       climbb.set(0);
//     }
    

//   }
//   public void setThePoint(double goal){
//     point= goal;
    
//     // return Math.abs(point-climbb.getRotorPosition().getValueAsDouble())<OperatorConstants.kElDeadBand;//should be changed
//   }
//   // public void rise(){
//   //   // climbb.set(10);
//   //   if(limSwitch.get()){
//   //     climbb.set(10);
    
//   //   }
//   //   else{
//   //     climbb.set(0);
     
//   //   }
//   // }
//   public void descend(){

//       climbb.set(10);
    
//     }
//   //   // else{
//   //   //   climbb.set(0);
     
//   //   // }
//   //   climbb.set(-10);
//   // }
//   public void stopClimb(){
//     climbb.set(0);
//   }
//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//     // System.out.println(limSwitch.get());
//   }
// }
