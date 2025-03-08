// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.AlgieArm;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class aDown extends Command {
//   /** Creates a new aDown. */
//     AlgieArm uppieDownniee;
//   public aDown(AlgieArm uppieDownnie) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     uppieDownniee = uppieDownnie;
//     addRequirements(uppieDownnie);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     uppieDownniee.setSetPoint(17);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {

//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     uppieDownniee.stopNeo();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return true;
//   }
// }
