package frc.robot.commands;

// import java.lang.annotation.Target;
// import java.security.spec.ECPublicKeySpec;
// import java.util.function.BooleanSupplier;
// import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.*;
// import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.*;
// import frc.robot.util.AllianceFlipUtil;

public class Neural_cmd extends Command {
    private final SwerveSubsystem swerveSubsystem ;
    private final Limelight_Neural limelight ;
    private final PIDController xController;
    private final PIDController aController;
    private final double targetClassID; 
    private final double targetDistance; 
   

    private static final double TX_THRESHOLD = 2.0; 
    private static final double DISTANCE_THRESHOLD = 0.18; // 距離閾值（公尺）
    private static final double MAX_SPEED = 1.5; // 最大平移速度（m/s）
    private static final double MAX_ANGULAR_SPEED = 2.0; // 最大角速度（rad/s）
    private static final double ALIGNMENT_THRESHOLD = 1.0 ;
    private static final double Constant_Distance = 0.3; // 假設校準後
    private static final double FILTER_ALPHA = 0.1;
    private double taFiltered = 0.0;
    private double txFiltered = 0.0;
    private boolean isAligned = false; // 追蹤對準狀態


    public Neural_cmd(SwerveSubsystem swerveSubsystem , Limelight_Neural limelight ,double targetClassID, double targetDistance ){
        this.swerveSubsystem = swerveSubsystem;
        this.limelight = limelight;
        this. targetClassID = targetClassID;
        this. targetDistance = targetDistance;
        this.xController = new PIDController(1 , 0 , 0);
        this.aController = new PIDController(0.7, 0, 0);
        addRequirements(limelight , swerveSubsystem);
        
        
    }

    public void initialize() {
        limelight.setLightMode(LimelightModule.LightMode.On); 
        xController.setSetpoint(0.0); 
        aController.setSetpoint(targetDistance);
        isAligned = false;
        SmartDashboard.putString("DriveToTarget_status", "Started");
    }

    @Override  
    public void  execute(){
       
        if (!limelight.hasValidTarget() || limelight.getId() != targetClassID) {
            swerveSubsystem.drive(new Translation2d(), 0.0, true, false);
            SmartDashboard.putString("DriveToTarget_status", "No valid target or wrong class");
            return;
        }
       
        double tx = limelight.getTx();
        // double ty = limelight.getTy();
        double ta = limelight.getTa();
        // txFiltered = FILTER_ALPHA * tx + (1 - FILTER_ALPHA) * txFiltered;
        txFiltered = tx;
        taFiltered = FILTER_ALPHA * ta + (1 - FILTER_ALPHA) * taFiltered;
        double distance = Constant_Distance / Math.sqrt(taFiltered);
       
        if (Math.abs(txFiltered) > ALIGNMENT_THRESHOLD) {
            isAligned = false;
        } else {
            isAligned = true;
        }

        double angularSpeed = 0;
        double forwardSpeed = 0;
        if (isAligned) {
            forwardSpeed = aController.calculate(distance);
        } else {
            angularSpeed = 
            xController.calculate(txFiltered);
            forwardSpeed = -aController.calculate(distance);
        }
        angularSpeed = Math.copySign(Math.min(Math.abs(angularSpeed), MAX_ANGULAR_SPEED), angularSpeed);
        forwardSpeed = Math.copySign(Math.min(Math.abs(forwardSpeed), MAX_SPEED), forwardSpeed);
        Translation2d translation = new Translation2d(-forwardSpeed, 0.0); 
        swerveSubsystem.drive( translation, angularSpeed, false, false); 

        SmartDashboard.putNumber("Raw_tx", tx);
        SmartDashboard.putNumber("Filtered_tx", txFiltered);
        SmartDashboard.putNumber("Raw_ta", ta);
        SmartDashboard.putNumber("Filtered_ta", taFiltered);
        SmartDashboard.putNumber("distance", distance);
        SmartDashboard.putBoolean("isAligned", isAligned);
        double distanceError = Math.abs(distance - targetDistance);
        SmartDashboard.putNumber("distanceerror", distanceError);
    }





    @Override

   public boolean isFinished() {
        if (!limelight.hasValidTarget()) {
            return true; 
        }
        double tx = Math.abs(limelight.getTx());
        double ta = limelight.getTa();
        if(isAligned){
            double distance = Constant_Distance / Math.sqrt(ta);
            double distanceError = Math.abs(distance - targetDistance);

            return tx < TX_THRESHOLD && distanceError < DISTANCE_THRESHOLD;
           
        }else{  
            return false;
        }
        
        
    }
    @Override
    public void end(boolean interrupted) {
        // limelight.setLightMode(LimelightModule.LightMode.Off);
        // swerveSubsystem.stopModules();
        swerveSubsystem.drive(new Translation2d(), 0.0, false, false);
    }

    // @Override
    // public boolean isFinished() {
    //     return finished;
    // }
}