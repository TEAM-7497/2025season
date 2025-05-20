// /*
//  * https://youtu.be/0Xi9yb1IMyA
//  * learn by FRC 0 to Autonomous: #6 Swerve Drive Auto
//  * thanks team 6814
//  */
// package frc.robot.commands;

// import java.util.function.Supplier;
// import java.util.function.BooleanSupplier;
// import java.util.function.IntSupplier;

// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.ShooterSubsystem;


// public class Auto_shoot extends Command {

//     private final ShooterSubsystem shooterSubsystem;


   
//     // private final Supplier<Double> ly;
//     // private final Supplier<Integer> pov;
//     // double kP = 0.5;
//     // double err = 0;
//     // double output = 0;
//     // double high = 0;
//     // double lastTime = System.currentTimeMillis() / 1000.0;
//     // double kI = 0.18;


//     public Auto_shoot(ShooterSubsystem shooterSubsystem
//     ) {
//         this.shooterSubsystem = shooterSubsystem;

//         addRequirements(shooterSubsystem);
//     }

//     @Override
//     public void initialize() {

//     }

//     @Override
//     public void execute() {
      

//         shooterSubsystem.coralshoot();
        
       



      





//     }

//     @Override
//     public void end(boolean interrupted) {
//         shooterSubsystem.stop();
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
        
//     }
// }
