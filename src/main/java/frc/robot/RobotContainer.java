// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlgieArmCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.elevatorCommand;
import frc.robot.commands.toFloor;
import frc.robot.commands.aDown;
import frc.robot.commands.aUp;
import frc.robot.commands.cDown;
import frc.robot.commands.coralArmCommand;
import frc.robot.commands.toL1;
import frc.robot.commands.toL2;
import frc.robot.commands.toL3;
import frc.robot.commands.toL4;
import frc.robot.commands.cUp;
import frc.robot.commands.climberDescend;
import frc.robot.commands.climberRise;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgieArm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.filter.SlewRateLimiter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.math.geometry.Rotation2d;

import static edu.wpi.first.units.Units.*;

// import com.pathplanner.lib.auto.AutoBuilder;

import frc.robot.subsystems.limelight;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;


import frc.robot.subsystems.aClaw;
import frc.robot.subsystems.cArm;
import frc.robot.subsystems.cClaw;
import frc.robot.subsystems.climber;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
        private double speed = OperatorConstants.kSpeed;
        public static cClaw corClaw = new cClaw();
       public static aClaw alClaw = new aClaw();
        private final limelight lime = new limelight();
        private final Elevator el = new Elevator();
       private final AlgieArm alArm = new AlgieArm();
      private final cArm coralArmm = new cArm();
      private final climber climb = new climber();
            private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)* OperatorConstants.kSpeed; // kSpeedAt12Volts desired top speed
            private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
        
            /* Setting up bindings for necessary control of the swerve drive platform */
            private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
            private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
            private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
        
            private final Telemetry logger = new Telemetry(MaxSpeed);
              private SlewRateLimiter slewLimY = new SlewRateLimiter(1.5);
          private SlewRateLimiter slewLimX = new SlewRateLimiter(1.5);
        
           
            private final CommandXboxController OpController = new CommandXboxController(OperatorConstants.kOpControllerPort);
            public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
          
          private final SwerveRequest.FieldCentricFacingAngle driveFacing = new SwerveRequest.FieldCentricFacingAngle()
              .withDeadband(MaxSpeed * 0.05).withDriveRequestType(DriveRequestType.OpenLoopVoltage);
          private final PhoenixPIDController steerController = new PhoenixPIDController(3, 0, 0.05);
          private final PhoenixPIDController steerController180 = new PhoenixPIDController(0.3, 0.001, 0.01);
          private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    
    driveFacing.HeadingController = steerController;
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() ->
            drive.withVelocityX(slewLimX.calculate(joyLeftY())* MaxSpeed*speedScale()) // Drive forward with negative Y (forward)
                .withVelocityY(-joyLeftX() * MaxSpeed*speedScale()) // Drive left with negative X (left)
                .withRotationalRate(-joyRightX()* MaxAngularRate) // Drive counterclockwise with negative X (left)
        )
    );
    el.setDefaultCommand(new elevatorCommand(el));
    alArm.setDefaultCommand(new AlgieArmCommand(alArm));
    coralArmm.setDefaultCommand(new coralArmCommand(coralArmm));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    driverController.leftBumper().whileTrue(drivetrain.applyRequest(() -> brake));
    // driverController.b().whileTrue(drivetrain.applyRequest(() ->
    //     point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
    // ));

    
    driverController.rightTrigger().whileTrue(new InstantCommand(()->speed= OperatorConstants.fastSpeed));
    driverController.leftTrigger().whileTrue(new InstantCommand(()->speed= OperatorConstants.slowSpeed));
    driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    driverController.start().onTrue(drivetrain.runOnce(()-> drivetrain.seedFieldCentric()));

     OpController.leftTrigger().whileTrue(new InstantCommand(()->corClaw.solenoidToggle()));//opens close
        OpController.rightTrigger().whileTrue(new InstantCommand(()->corClaw.turn()));
        OpController.x().whileTrue(new InstantCommand(()->alClaw.closeClawA()));
    driverController.rightBumper().whileTrue(new toFloor(el));

        driverController.a().whileTrue(new toL1(el));
        driverController.x().whileTrue(new toL2(el));
        driverController.y().whileTrue(new toL3(el));
        driverController.b().whileTrue(new toL4(el));
        driverController.leftBumper().whileTrue(new aUp(alArm));
        driverController.rightBumper().whileTrue(new aDown(alArm));

      OpController.leftBumper().whileTrue(new cUp(coralArmm)); // don't have enough button
      OpController.rightBumper().whileTrue(new cDown(coralArmm));//same 
      OpController.back().whileTrue(new climberRise(climb));
      OpController.start().whileTrue(new climberDescend(climb));
      OpController.a().whileTrue(new aUp(alArm));
      OpController.y().whileTrue(new aDown(alArm));
      
    drivetrain.registerTelemetry(logger::telemeterize);  

  }
  public double joyRightX(){
    double rightX = driverController.getRightX();
    if(Math.abs(rightX)> OperatorConstants.kJoyRightXDeadzone){
        return rightX;
    }
    return 0;
}
public double joyLeftX(){
    double leftX = driverController.getLeftX();
    if(Math.abs(leftX) > OperatorConstants.kJoyLeftXDeadzone){
        return leftX;
    }
    return 0;
}
public double joyLeftY(){
    double  leftY= driverController.getLeftY();
    if(Math.abs(leftY) > OperatorConstants.kJoyLeftYDeadzone){
        return leftY;
    }
    return 0;
}
public double speedScale(){
  if(driverController.leftBumper().getAsBoolean())
  return Constants.OperatorConstants.fastSpeed;
  if(driverController.rightBumper().getAsBoolean())
  return Constants.OperatorConstants.slowSpeed;
  
  return Constants.OperatorConstants.normalSpeed;
}
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
