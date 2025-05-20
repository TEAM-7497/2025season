package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private SwerveSubsystem s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private DoubleSupplier pl;
    private DoubleSupplier pr;
    private BooleanSupplier FieldOriented;
    private BooleanSupplier RobotOriented;
    private Boolean DriveModeField = true;
    // private double rotationVal = 0;
    // private double translationVal = 0;
    // private double strafeVal = 0;

    public TeleopSwerve(SwerveSubsystem s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup,  
                        DoubleSupplier pl, DoubleSupplier pr, BooleanSupplier FieldOriented, BooleanSupplier RobotOriented) {

        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.pl = pl;
        this.pr = pr;
        this.FieldOriented = FieldOriented;
        this.RobotOriented = RobotOriented;
    }

    @Override
    public void execute() {
        if(FieldOriented.getAsBoolean() == true){
            DriveModeField = true;
        }
        if(RobotOriented.getAsBoolean() == true){
            DriveModeField = false;
        }
        /* Get Values, Deadband*/
        // if (PreciseControl_Right.getAsDouble() != 0 || PreciseControl_Left.getAsDouble() != 0) {
        //     double strafeVal = (PreciseControl_Right.getAsDouble() - PreciseControl_Left.getAsDouble())/2;
        //     double translationVal = 0;
        //     double rotationVal = 0;
        // }else{
            double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
            double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
            double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

            if (pr.getAsDouble() != 0 || pl.getAsDouble() != 0) {
                s_Swerve.drive(
                    new Translation2d(0, (pr.getAsDouble()-pl.getAsDouble())/4), 
                    0, 
                    false,   
                    true
                );
            }else{
                    if(DriveModeField){
                    s_Swerve.drive(
                        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
                        rotationVal * Constants.Swerve.maxAngularVelocity, 
                        true, 
                        true
                    );
                }else{
                    s_Swerve.drive(
                        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
                        rotationVal * Constants.Swerve.maxAngularVelocity, 
                        false, 
                        true
                    );
                }
        }
    }
}