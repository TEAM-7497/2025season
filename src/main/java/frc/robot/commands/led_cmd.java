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
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class led_cmd extends Command {

    private final LedSubsystem ledSubsystem;
    // private final ShooterSubsystem shooterSubsystem;
    double kP = 0.43;
    double err = 0;
    double output = 0;
    double high = 0;
    double lastTime = System.currentTimeMillis() / 1000.0;
    double kI = 0.18;


    public led_cmd(LedSubsystem ledSubsystem
    ) {
        this.ledSubsystem = ledSubsystem;

        addRequirements(ledSubsystem);
        // addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        
        





    }

    @Override
    public void end(boolean interrupted) {


    }

    @Override
    public boolean isFinished() {
        return false;
        
    }
}
