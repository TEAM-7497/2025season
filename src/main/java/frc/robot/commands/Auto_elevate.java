/*
 * https://youtu.be/0Xi9yb1IMyA
 * learn by FRC 0 to Autonomous: #6 Swerve Drive Auto
 * thanks team 6814
 */
package frc.robot.commands;

import java.util.function.Supplier;
import java.util.function.IntSupplier;
import java.lang.Math;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Auto_elevate extends Command {

    private final ElevatorSubsystem elevatorSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    double kP = 0.45;
    double err = 0;
    double output = 0;
    double high = 0;
    double lastTime = System.currentTimeMillis() / 1000.0;
    double kI = 0.18;
    int floor;
    Timer timer = new Timer();
    static boolean finished=false;


    public Auto_elevate(ElevatorSubsystem elevatorSubsystem, ShooterSubsystem shooterSubsystem,int floor
    ) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.floor = floor;

        addRequirements(elevatorSubsystem);
        addRequirements(shooterSubsystem);
  
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        elevatorSubsystem.show();

            if(floor == 0){
                elevatorSubsystem.slowdown();
            }else if (floor == 1){
                high = 3.6;
            }else if (floor == 2){
                high = 6.3;
            }else if(floor == 3){
                high = 10.2;
            }else if(floor == 4){
                high = 17.7;
            }

            
        if(floor != 0){
            err = high - elevatorSubsystem.getPosition();
            output = err * kP;
            if(output < 0.2 && output > -0.2){
                elevatorSubsystem.stop();
            }else if(output >= 4.5){ 
                elevatorSubsystem.set(4.5);
            }else{
                elevatorSubsystem.set(output);
            }
            
            if(Math.abs(Math.abs(elevatorSubsystem.getPosition())-high)<2.25){
                shooterSubsystem.coralshoot();
            }else{
                    SmartDashboard.putBoolean("shoot", false);
                }
            
            SmartDashboard.putNumber("eleelele", elevatorSubsystem.getPosition());
            SmartDashboard.putNumber("offfffsetfff", Math.abs(Math.abs(elevatorSubsystem.getPosition())-high));
        }
        


        // SmartDashboard.putNumber("err",err);
        // SmartDashboard.putNumber("high",high);
        // SmartDashboard.putNumber("output",output);




    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stop();


    }

    
    public static void finish(){
        finished=true;
    }

    @Override
    public boolean isFinished() {
        return finished;
        
    }
}
