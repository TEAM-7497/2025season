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


// public class Align_auto extends Command {

//     private final SwerveSubsystem swerveSubsystem;
//     private final PIDController xController;
//     private final PIDController yController;
//     private final PIDController thetaController;
//     private final LimelightLeft limelightLeft;
//     private boolean finished;
//     private double rotationSpeed;
//     private double rSpeed;
//     private double ySpeed;
//     private double xSpeed;
//     private double lock;
//     private int side;
//     Translation2d translation;

//     Timer timer;
//     private boolean xok;

//     public Align_auto(SwerveSubsystem swerveSubsystem, LimelightLeft limelightLeft, int side) {
//         this.swerveSubsystem = swerveSubsystem;
//         this.limelightLeft = limelightLeft;
//         this.xController = new PIDController(3.3, 0, 0);
//         this.yController = new PIDController(3.3, 0, 0);
//         this.thetaController = new PIDController(0.16, 0, 0);
//         this.side = side;

//     }

//     @Override
//     public void initialize() {
//         finished = false;
//         xok = false;
//         timer = new Timer();
//         timer.reset();
//     }

//     @Override
//     public void execute() {
//                 if(limelightLeft.tagdect() ==0){
//                     finished = true;
//                 }

//                 double[] targetTagPose = limelightLeft.getAprilTag();
//                 double xOffset = -targetTagPose[0];
//                 double yOffset = targetTagPose[2];
//                 double rOffset = targetTagPose[4];
//                 SmartDashboard.putNumber("xxxxx", Math.abs(xOffset)-0.2);
//                 SmartDashboard.putNumber("yyyyy", yOffset);
//                 SmartDashboard.putNumber("rrrrr", rOffset);
                
                
//                 if (side == 0) {  
//                     // left
//                     timer.reset();
//                     timer.start();
//                     xok = false;
//                     lock = xController.calculate(0, 0);
//                     xSpeed = xController.calculate(xOffset, -0.165); 
//                     ySpeed = yController.calculate(yOffset, 0); 
//                     rSpeed = thetaController.calculate(rOffset, 0);
//                     rotationSpeed = rSpeed;
//                     SmartDashboard.putNumber("xspeed",xSpeed);
//                     SmartDashboard.putNumber("yspeed",ySpeed);
//                     SmartDashboard.putNumber("rspeed",rSpeed);


//                     translation = new Translation2d(ySpeed, xSpeed); 
//                     SmartDashboard.putNumber("mode", 2);
//                     SmartDashboard.putNumber("time", timer.get());
//                     if(Math.abs(yOffset)<=0.01 && Math.abs(xOffset+0.165)<=0.01){
//                         finished = true;
//                         SmartDashboard.putBoolean("finisheddd", true);
//                     }
//                     swerveSubsystem.drive(translation, -rotationSpeed, false, false);
                    
//                     // if(xSpeed>=0.5) xSpeed=0.5;
//                     // if(ySpeed>=0.5) ySpeed=0.5;
//                     // if(rotationSpeed>=1) rotationSpeed=1;
//                     // if(Math.abs((xOffset+0.22)) <= 0.08 && Math.abs(rOffset) <= 2){
//                     //     xok = true;
//                     // }
                    
//                     // if(timer.get()>0.6){
//                     //     translation = new Translation2d(lock, xSpeed); 
//                     //     rotationSpeed = rSpeed;     
//                     //     SmartDashboard.putNumber("mode", 1);
//                     //     SmartDashboard.putNumber("time", timer.get());
//                     // }else{
//                     // }
//                     // if(!xok && timer.get()>=0.6){
//                     //     translation = new Translation2d(ySpeed, xSpeed); 
//                     //     rotationSpeed = rSpeed;       
//                     //     SmartDashboard.putNumber("mode", 2);
//                     //     SmartDashboard.putNumber("time", timer.get());

//                     // }
//                     // if(xok){
//                     //     translation = new Translation2d(ySpeed, lock); 
//                     //     rotationSpeed = lock;       
//                     //     if(Math.abs(yOffset)<=0.06){
//                     //         finished = true;
//                     //     }
//                     //     SmartDashboard.putNumber("mode", 3);
//                     //     SmartDashboard.putNumber("time", timer.get());


//                     // }

                    
//                 } else if (side == 1) {
//                     // righ
//                     timer.reset();
//                     timer.start();
//                     xok = false;
//                     lock = xController.calculate(0, 0);
//                     xSpeed = xController.calculate(xOffset, 0.165); 
//                     ySpeed = yController.calculate(yOffset, 0); 
//                     rSpeed = thetaController.calculate(rOffset, 0);
//                     rotationSpeed = rSpeed;
//                     SmartDashboard.putNumber("xspeed",xSpeed);
//                     SmartDashboard.putNumber("yspeed",ySpeed);
//                     SmartDashboard.putNumber("rspeed",rSpeed);
//                     translation = new Translation2d(ySpeed, xSpeed); 
//                     rotationSpeed = rSpeed;       
//                     SmartDashboard.putNumber("mode", 2);
//                     SmartDashboard.putNumber("time", timer.get());
//                     if (Math.abs(yOffset) <= 0.01 && Math.abs(xOffset-0.165) <= 0.01){
//                         finished = true;
//                         SmartDashboard.putBoolean("finisheddd", true);
//                     }
//                     swerveSubsystem.drive(translation, -rotationSpeed, false, false);
//             }
//         }



//     public void finish(){
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
