// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;
// import com.ctre.phoenix6.swerve.SwerveDrivetrain;

// import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.networktables.DoublePublisher;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.networktables.StructArrayPublisher;
// import edu.wpi.first.networktables.StructPublisher;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// /** Add your docs here. */


// public class poseEstimator {
   
//     SwerveDrivePoseEstimator estimate;
//      private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

//     /* Robot swerve drive state */
//     private final NetworkTable driveStateTable = inst.getTable("DriveState");
//     private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();



//     public poseEstimator(CommandSwerveDrivetrain drive ){
       
//         estimate = new SwerveDrivePoseEstimator(drive.getKinematics(),drive.getPigeon2().getRotation2d(),dr);
//         estimate = new SwerveDrivePoseEstimator(null, null, null, null, null, null);
//     }

    
// }
