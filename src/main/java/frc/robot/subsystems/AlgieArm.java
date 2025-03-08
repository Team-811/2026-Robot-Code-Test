// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;

public class AlgieArm extends SubsystemBase {
  /** Creates a new AlgieArm. */
   private SparkMax uppieDownnie;
    private SparkMaxConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
 private RelativeEncoder encoder;
 private double setPoint;
  public AlgieArm() {
    uppieDownnie = new SparkMax(OperatorConstants.neoId, MotorType.kBrushless);

    closedLoopController = uppieDownnie.getClosedLoopController();
    encoder = uppieDownnie.getEncoder();
    motorConfig = new SparkMaxConfig();
        motorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

            motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.05)
        .i(0)
        .d(0);
        // .outputRange(-1, 1);
        // // Set PID values for velocity control in slot 1
        // .p(0.0001, ClosedLoopSlot.kSlot1)
        // .i(0, ClosedLoopSlot.kSlot1)
        // .d(0, ClosedLoopSlot.kSlot1)
        // .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        // .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

         uppieDownnie.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void turnPoint(){
    // System.out.println(elMotor.getRotorPosition().getValue());
    if(Math.abs(setPoint-uppieDownnie.getEncoder().getPosition())>1){
      closedLoopController.setReference(setPoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
    else{
      uppieDownnie.set(0);
    }
    }
    public void setSetPoint(double goal){
        setPoint= goal;
      }
  public void stopNeo(){
    uppieDownnie.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
