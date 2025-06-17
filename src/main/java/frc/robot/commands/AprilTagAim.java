package frc.robot.commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.limelight;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.units.Units.*;

public class AprilTagAim extends Command {
  limelight limee;
  CommandSwerveDrivetrain drivetrainie;
  Timer timer;
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  public AprilTagAim(limelight lime, CommandSwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
   
   limee = lime;
    drivetrainie = drivetrain;
    timer = new Timer();
    addRequirements(limee, drivetrainie);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(limee.hasTarget()){
        timer.reset();
    }

        drivetrainie.applyRequest(() ->
    new SwerveRequest.RobotCentric().withVelocityX(limee.RobotXDutyCycle() * 3) // Drive forward with negative Y (forward)
   
    .withVelocityY(limee.RobotYDutyCycle()* 3) // Drive left with negative X (left)
        .withRotationalRate(
          
        limee.AimTargetYawDutyCycle()
          
          * MaxAngularRate) // Drive counterclockwise with negative X (left)
).execute();
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainie.applyRequest(() ->
    new SwerveRequest.RobotCentric().withVelocityX(0) // Drive forward with negative Y (forward)
    .withVelocityY(0) // Drive left with negative X (left)
    .withRotationalRate(0) // Drive counterclockwise with negative X (left)
).execute();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return 
    (Math.abs(limee.targetXError()) < 0.0041 //0.05, 0.0041
    && Math.abs(limee.targetZError()) < 0.015 //0.108, 0.015
    && Math.abs(limee.targetYawError()) < 0.025//0.05, 0.025
    ) || timer.hasElapsed(1);
  }
}


