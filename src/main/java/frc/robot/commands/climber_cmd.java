// /*
//  * https://youtu.be/0Xi9yb1IMyA
//  * learn by FRC 0 to Autonomous: #6 Swerve Drive Auto
//  * thanks team 6814
//  */
// package frc.robot.commands;

// import java.util.function.Supplier;
// import java.util.function.IntSupplier;

// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;

// import frc.robot.subsystems.ClimberSubsystem;

// public class climber_cmd extends Command {

//     private final ClimberSubsystem  climberSubsystem;
//     private final Supplier<Double> rx;
    
//     double kP = 0.5;


//     public climber_cmd(ClimberSubsystem climberSubsystem, Supplier<Double> rx
//     ) {
//         this.climberSubsystem = climberSubsystem;
//         this.rx = rx;
    
//         addRequirements(climberSubsystem);
//     }

//     @Override
//     public void initialize() {

//     }

//     @Override
//     public void execute() {
    
//         if (rx.get() < 0.7 && rx.get() > -0.7) {
//             climberSubsystem.stop();
//         } else if (rx.get() <= -0.7){
//             climberSubsystem.down();
//         } else {
//             climberSubsystem.up();
//         }
//     }

//     @Override
//     public void end(boolean interrupted) {

//     }

//     @Override
//     public boolean isFinished() {
//         return false;
        
//     }
// }
