/*
 * https://youtu.be/0Xi9yb1IMyA
 * learn by FRC 0 to Autonomous: #6 Swerve Drive Auto
 * thanks team 6814
 */
package frc.robot.commands;

import java.util.function.Supplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Elevator_cmd extends Command {

    private final ElevatorSubsystem elevatorSubsystem;
    // private final ShooterSubsystem shooterSubsystem;
    private final Supplier<Double> ly;
    private final Supplier<Double> lx;
    // private final Supplier<Double> ry;
    private final Supplier<Integer> pov;
    double kP = 0.43;
    double err = 0;
    double output = 0;
    double high = 0;
    double lastTime = System.currentTimeMillis() / 1000.0;
    double kI = 0.18;


    public Elevator_cmd(ElevatorSubsystem elevatorSubsystem,   Supplier<Double> ly, Supplier<Double> lx,Supplier<Integer> pov
    ) {
        this.elevatorSubsystem = elevatorSubsystem;
        // this.shooterSubsystem = shooterSubsystem;
        this.lx = lx;
        this.ly = ly;
        this.pov = pov;

        addRequirements(elevatorSubsystem);
        // addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        elevatorSubsystem.show();
        if(ly.get() > 0.5){
            elevatorSubsystem.up();
        }else if (ly.get() < -0.5){
            elevatorSubsystem.down();
        }else if(lx.get() > 0.5){
            elevatorSubsystem.TurnRight();
        }else if(lx.get() < -0.5){
            elevatorSubsystem.TurnLeft();
        }else{
            elevatorSubsystem.stop();
        }
        
        // if(pov.get() == -1){

        //     if (ly.get() < 0.7 && ly.get() > -0.7) {
        //         elevatorSubsystem.stop();
        //         //elevatorSubsystem.holdPosition();
        //     } else if (ly.get() <= -0.7) {
        //         elevatorSubsystem.up();
        //     } else {
        //         elevatorSubsystem.down();
        //     }

           


        // }else{
        //     // shooterSubsystem.setangle(5.5);
        //     if (pov.get() == 0){   
                
        //         high = 3.5;
        //     }else if (pov.get() == 90){
                
        //         high = 5.9;
        //     }else if(pov.get() == 180){

        //         high = 9.8;
        //     }else if(pov.get() == 270){
              
        //         high = 0;
        //     }
            
        //     err = high - elevatorSubsystem.getPosition();
        //     if(pov.get() == 270){
        //         output = err * 0.2;
        //     }else{
        //         output = err * kP;
        //     }
        //     if(output < 0.2 && output > -0.2){
        //         elevatorSubsystem.stop();
        //     }else if(output >= 3.9){ 
        //         elevatorSubsystem.set(3.9);
        //     }
        //     else if(output <= -1.5){
        //         elevatorSubsystem.set(-1.5);
        //     }
        //     else{
        //         elevatorSubsystem.set(output);
        //     }
        // }

        // SmartDashboard.putNumber("err",err);
        // SmartDashboard.putNumber("high",high);
        // SmartDashboard.putNumber("output",output);





    }

    @Override
    public void end(boolean interrupted) {


    }

    @Override
    public boolean isFinished() {
        return false;
        
    }
}
