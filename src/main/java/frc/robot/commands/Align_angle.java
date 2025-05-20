// package frc.robot.commands;

// import java.lang.annotation.Target;
// import java.util.function.BooleanSupplier;
// import java.util.function.Supplier;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.units.measure.Time;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.*;
// import edu.wpi.first.wpilibj.Timer;
// import frc.robot.subsystems.*;


// public class Align_angle extends Command {

//     private final SwerveSubsystem swerveSubsystem;
//     private final PIDController thetaController;
//     private final Limelight limelight;
//     private static boolean finished = false;
//     private double rotationSpeed;
//     private double ySpeed;
//     private double xSpeed;
//     private double lock;
//     Timer timer = new Timer();

//     public Align_angle(SwerveSubsystem swerveSubsystem, Limelight limelight) {
//         this.swerveSubsystem = swerveSubsystem;
//         this.limelight = limelight;
//         this.thetaController = new PIDController(0.055, 0, 0);
//     }

//     @Override
//     public void execute() {
    
//         double[][] detectedTags = limelight.getAllTagPoses();

//         if (detectedTags.length == 0) {
//             swerveSubsystem.stopModules();
//             finished = true;    
//         }
//         double[] closestTagPose = null;
//         double minDistance = Double.MAX_VALUE;

//         for (double[] tagPose : detectedTags) {
//             // double xOffset = tagPose[0];
//             double yOffset = tagPose[2];
//             // double distance = Math.sqrt(xOffset * xOffset + yOffset * yOffset);

//             if (yOffset < minDistance) {
//                 minDistance = yOffset;
//                 closestTagPose = tagPose;
                
//             }
//         }

//         if (closestTagPose != null) {


//                     double rOffset = closestTagPose[4];
//                     lock = thetaController.calculate(0,0);
//                     rotationSpeed = thetaController.calculate(rOffset, 0);
//                     Translation2d translation = new Translation2d(lock, lock);        
//                     swerveSubsystem.drive(translation, -rotationSpeed,false, false);}
                        
//                 }

//     public static void finish(){
//         finished = true;
//     }

//     @Override
//     public void end(boolean interrupted) {
//         swerveSubsystem.stopModules();
//     }

//     @Override
//     public boolean isFinished() {
//         return finished;
//     }
// }
