// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.cArm;
import frc.robot.subsystems.rollerClaw;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class cL4x3 extends ParallelCommandGroup {
  /** Creates a new cL4x3. */
  public cL4x3(Elevator el, CommandSwerveDrivetrain drivetrain, rollerClaw rolly, String path, cArm coArm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new cL4x2(el, drivetrain, rolly, path, coArm),new PathPlannerAuto("midL4x3"), new toL4(el).andThen(new IntakeCommand(rolly)));
  }
}
