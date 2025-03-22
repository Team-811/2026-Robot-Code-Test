// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OperatorConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.limelight;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class getHorizontalOffset extends Command {
  /** Creates a new getHorizontalOffset. */
private CommandSwerveDrivetrain drivetrain;

           private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)* OperatorConstants.kSpeed;
           private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  private final RobotCentric robotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed*0.05).withRotationalDeadband(MaxAngularRate*0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  public getHorizontalOffset(CommandSwerveDrivetrain drivetrainn) {
    // // Use addRequirements() here to declare subsystem dependencies.
    // LimelightHelpers.getTX("limelight");
    drivetrain= drivetrainn;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println(   LimelightHelpers.getTX("limelight"));
    // System.out.println("hello");

    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    NetworkTable lime  = instance.getTable("limelight");
    double[] valid = lime.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);

    long targetId = lime.getEntry("tid").getInteger(0);
    // System.out.println(valid);
    System.out.println(valid[0]);
    System.out.println(valid[5]);
    System.out.println("id: "+targetId);

    drivetrain.setDefaultCommand(
      // Drivetrain will execute this command periodically
      drivetrain.applyRequest(()->robotCentric
          .withVelocityX(valid[0]) // Drive forward with negative Y (forward)
              .withVelocityY(0) // Drive left with negative X (left)
      )
  );
    LimelightResults results = LimelightHelpers.getLatestResults("limelight");
    // System.out.println(results.valid);
    // System.out.println(results.targets_Fiducials.length);
    // System.out.println(results.error);
    if(results.valid && results.targets_Fiducials.length>0){
      LimelightTarget_Fiducial tag = results.targets_Fiducials[0];

      Pose3d tagPose = tag.getTargetPose_CameraSpace();
      System.out.println(tagPose.getX());
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
