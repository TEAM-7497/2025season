/*
 * https://youtu.be/0Xi9yb1IMyA
 * learn by FRC 0 to Autonomous: #6 Swerve Drive Auto
 * thanks team 6814
 */
package frc.robot.commands;

import java.util.function.Supplier;
import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeSubsystem;

public class intake_cmd extends Command {

    private final IntakeSubsystem  intakeSubsystem;
    private static boolean frist = true;
    private final Supplier<Double> ry;
    private final Supplier<Boolean> y;
;
    double kP = 0.5;
    double err = 0;
    double output = 0;
    double high = 0;
    double lastTime = System.currentTimeMillis() / 1000.0;
    double kI = 0.18;

    // private final Supplier<Double> ly;
    // private final Supplier<Integer> pov;
    // double kP = 0.5;
    // double err = 0;
    // double output = 0;
    // double high = 0;
    // double lastTime = System.currentTimeMillis() / 1000.0;
    // double kI = 0.18;


    public intake_cmd(IntakeSubsystem intakeSubsystem ,  Supplier<Double> ry, Supplier<Boolean> y
    ) {
        this.intakeSubsystem = intakeSubsystem;
        this.ry = ry;
        this.y = y;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        frist = false;

    }

    @Override
    public void execute() {
      
    //     if(lx.get() > 0.2){
    //         intakeSubsystem.setsuck(4);
    //    }else if( lx.get() < -0.2){
    //        intakeSubsystem.setsuck(-3);
    //    }else{
    //         intakeSubsystem.setsuck(0);
    //    }
        intakeSubsystem.showencoder();

       if(ry.get() > 0.7){
        intakeSubsystem.angleout(-1.5);
        }else if(ry.get() < -0.7){
            intakeSubsystem.angleout(2.3);
        }else{
            intakeSubsystem.angleout(0.5);
        }

        if (y.get()) {
            intakeSubsystem.setangle(-1.7);
            intakeSubsystem.algaesuck();
            frist = true;
        }else{
            if(frist){
                intakeSubsystem.setangle(0);
                intakeSubsystem.suckstop();
                if(Math.abs(intakeSubsystem.getEncoder())< 0.2){
                frist = false;
                }
            }
        }
    
        
       

//-----------------------------------------------------MONKEY
        
        // double errSum = 0.0;
        // if(errSum > 2){
        //     errSum = 0.0;
        // }
        // if(pov.get() == 0){
        //     double currentTime = System.currentTimeMillis() / 1000.0;
        //     double deltaTime = currentTime - lastTime;
        //     lastTime = currentTime;
        //     double err = 3.2 - elevatorSubsystem.getPosition();
        //     if(err < 2 && err > -2){
        //         errSum += err * deltaTime;
        //     }else{
        //         errSum = 0;
        //     }
        //     double output = err * kP  ;
        //     if(err < 2 && err > -2){
        //         elevatorSubsystem.stop();
        //     }else{
        //         elevatorSubsystem.set(-output);
        //     }
        //     elevatorSubsystem.setPosition(0);
        // }else if (pov.get() == 90){
        //     double currentTime = System.currentTimeMillis() / 1000.0;
        //     double deltaTime = currentTime - lastTime;
        //     lastTime = currentTime;
        //     double err = 3.2 - elevatorSubsystem.getPosition();
        //     if(err < 2 && err > -2){
        //         errSum += err * deltaTime;
        //     }else{
        //         errSum = 0;
        //     }
        //     double output = err * kP  ;
        //     if(err < 2 && err > -2){
        //         elevatorSubsystem.stop();
        //     }else{
        //         elevatorSubsystem.set(-output);
        //     }
        // }else if (pov.get() == 180){
        //     double currentTime = System.currentTimeMillis() / 1000.0;
        //     double deltaTime = currentTime - lastTime;
        //     lastTime = currentTime;
        //     double err = 4.9 - elevatorSubsystem.getPosition();
        //     if(err < 2 && err > -2){
        //         errSum += err * deltaTime;
        //     }else{
        //         errSum = 0;
        //     }
        //     double output = err * kP  ;
        //     if(err < 2 && err > -2){
        //         elevatorSubsystem.stop();
        //     }else{
        //         elevatorSubsystem.set(-output);
        //     }
        // }else if (pov.get() == 270){
            
        //     double currentTime = System.currentTimeMillis() / 1000.0;
        //     double deltaTime = currentTime - lastTime;
        //     lastTime = currentTime;
        //     double err = 14.7 - elevatorSubsystem.getPosition();
        //     if(err < 2 && err > -2){
        //         errSum += err * deltaTime;
        //     }else{
        //         errSum = 0;
        //     }
        //     double output = err * kP  ;
        //     if(err < 2 && err > -2){
        //         elevatorSubsystem.stop();
        //     }else{
        //         elevatorSubsystem.set(-output);
        //     }
        //     //level four
        // }else{
        //     if (ly.get() < 0.2 && ly.get() > -0.2) {
        //         elevatorSubsystem.stop();
        //     } else if (ly.get() <= -0.2) {
        //         elevatorSubsystem.down();
        //     } else {
        //         elevatorSubsystem.up();
        //     }
        //     // human 
        // }

//--------------------------------------------
        // }else if (pov.get()==45){
        //     double err = 10 - elevatorSubsystem.getPosition();
        //     double output = err * kP;
        //     elevatorSubsystem.set(output);
        // }else if (pov.get()==45){
        //     double err = 15 - elevatorSubsystem.getPosition();
        //     double output = err * kP;
        //     elevatorSubsystem.set(output);
        // }else if (pov.get()==45){
        //     double err = 20 - elevatorSubsystem.getPosition();
        //     double output = err * kP;
        //     elevatorSubsystem.set(output);
        // }



        // yiping below

        // if(ly.get()>0.5){
        //     elevatorSubsystem.up_left() ;
        //     elevatorSubsystem.up_right() ;
        // }
        // else if(ly.get()<-0.5){
        //     elevatorSubsystem.down_left() ;
        //     elevatorSubsystem.down_right() ;
        // }
        // else{
        //     elevatorSubsystem.stop_left() ;
        //     elevatorSubsystem.stop_right() ;
        // }
        // if(ly.get()>0.5){
        //     elevatorSubsystem.up() ;
           
        // }
        // else if(ly.get()<-0.5){
        //     elevatorSubsystem.down();
           
        // }
        // else{
        //     elevatorSubsystem.stop_tmp();
        // }
      






    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
        
    }
}
