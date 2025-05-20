package frc.robot;

import java.util.concurrent.CopyOnWriteArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import javax.print.attribute.standard.CopiesSupported;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;


import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Swerve;
import frc.robot.autos.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
//-----------------------
    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    // public final ClimberSubsystem climberSubsystem = new ClimberSubsystem(); 
    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    public final LedSubsystem ledSubsystem = new LedSubsystem();
    public final LimelightLeft limelightLeft = new LimelightLeft();
    public final LimelightRight limelightRight = new LimelightRight();
    public final LimelightModule module  = new LimelightModule();
    public final Limelight_Neural limelight_Neural = new Limelight_Neural(module);
    private final Field2d field;
     
    
     

    /* Controllers */
    private final Joystick driver = new Joystick(1);
    private final CommandXboxController copilotJoystick = new CommandXboxController(0);

    /* Drive Controls */
    private final int translationAxis = 1;
    private final int strafeAxis = 0;
    private final int rotationAxis = 4;
    private BooleanSupplier seeCoral = () -> limelight_Neural.hasValidTarget();
    
    private final int pl = 2;
    private final int pr = 3;
    private final int CoralId = 1;
    private final double CoralDistance = 0.07;
    private final SendableChooser<Command> autoChooser;
    private final PathConstraints STANDARD_CONSTRAINTS = new PathConstraints(
        2.5 , 1.5,
        Units.degreesToRadians(540) , Units.degreesToRadians(720)
    );
    private final Pose2d Reef1 = new Pose2d(10*0.3 , 14*0.3 , Rotation2d.fromDegrees(180));
    private final Pose2d Reef2 = new Pose2d(12*0.3 , 18*0.3 , Rotation2d.fromDegrees(300));
    private final Pose2d Reef3 = new Pose2d(10*0.3 , 14*0.3 , Rotation2d.fromDegrees(210));
 
    private final Pose2d coralStation = new Pose2d(5*0.3 , 4*0.3 , Rotation2d.fromDegrees(45));
    


    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton Left = new JoystickButton(driver, 5);
    private final JoystickButton Right = new JoystickButton(driver, 6);
    private final JoystickButton Mid = new JoystickButton(driver, XboxController.Button.kA.value);

    /* Subsystems */
    private final SwerveSubsystem s_Swerve = new SwerveSubsystem();


    /* The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        field = new Field2d();
        SmartDashboard.putData("Field", field);
        
        
        

        



        // NamedCommands.registerCommand("coral", new Auto_coral_cmd(elevatorSubsystem, shooterSubsystem, 2));
        NamedCommands.registerCommand("elevate_3", new Auto_elevate(elevatorSubsystem,shooterSubsystem,3).withTimeout(2));
        NamedCommands.registerCommand("elevate_2", new Auto_elevate(elevatorSubsystem,shooterSubsystem,2).withTimeout(1.8));
        NamedCommands.registerCommand("elevate_1", new Auto_elevate(elevatorSubsystem,shooterSubsystem,1).withTimeout(1.3));
        NamedCommands.registerCommand("elevate_0", new Auto_elevate(elevatorSubsystem,shooterSubsystem,0).withTimeout(0.8));
        NamedCommands.registerCommand("align_left", new Align_cmd(s_Swerve, limelightLeft, 0).withTimeout(3.6));
        NamedCommands.registerCommand("align_right", new Align_cmd(s_Swerve, limelightLeft, 1).withTimeout(3.6));
        
        // NamedCommands.registerCommand("shootSet", new autoSet_cmd(shooterSubsystem));
        // NamedCommands.registerCommand("shoot", new autoShoot(intakeSubsystem, shooterSubsystem));
        // NamedCommands.registerCommand("reset_gyro", Commands.runOnce(() -> swerveSubsystem.zeroHeading()));
        // NamedCommands.registerCommand("rotate_2_0", new rotate_cmd(swerveSubsystem, 0));
        // NamedCommands.registerCommand("rotate_2_120", new rotate_cmd(swerveSubsystem, 120));
        // NamedCommands.registerCommand("rotate_2_240", new rotate_cmd(swerveSubsystem, 240));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> driver.getRawAxis(translationAxis), 
                () -> driver.getRawAxis(strafeAxis), 
                () -> driver.getRawAxis(rotationAxis), 
                () -> driver.getRawAxis(pl),
                () -> driver.getRawAxis(pr),
                () -> driver.getRawButton(XboxController.Button.kB.value),
                () -> driver.getRawButton(XboxController.Button.kX.value)
            ));

        //-------------------------------joker

        // elevatorSubsystem.setDefaultCommand(new Elevator_cmd(elevatorSubsystem,
        // () -> copilotJoystick.getLeftY(),
        // // () -> copilotJoystick.getRightY(),
        // () -> copilotJoystick.getHID().getPOV()));

        // climberSubsystem.setDefaultCommand(new climber_cmd(climberSubsystem,
        // () -> copilotJoystick.getRightX()));

        intakeSubsystem.setDefaultCommand(new intake_cmd(intakeSubsystem,
        () -> copilotJoystick.getRightY(),
        copilotJoystick.y()::getAsBoolean//e04誰知道要這樣寫
        ));


        ledSubsystem.setDefaultCommand(new led_cmd(ledSubsystem));
        // intakeSubsystem.setDefaultCommand(new intake_cmd(intakeSubsystem,
        // () -> copilotJoystick.getLeftX()));

        // shooterSubsystem.setDefaultCommand(new Shooter_cmd(shooterSubsystem, 
        // () -> copilotJoystick.getRightBumper()));




        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    
    
    
     private void configureButtonBindings() {
        /* Driver Buttons */
        // zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        // Left.onTrue(new Align_cmd(s_Swerve, limelightLeft, 0).withTimeout(1.3));
        Right.whileTrue(new Neural_cmd(s_Swerve, limelight_Neural, CoralId, CoralDistance));
            // Mid.whileTrue(Commands.run(() -> s_Swerve.zeroGyro()));
        Left.whileTrue(Commands.defer(() ->AutoBuilder.pathfindToPose(
            Reef1,
            STANDARD_CONSTRAINTS,
            3.0), java.util.Set.of(s_Swerve) )
            .until(seeCoral)
            .andThen(new Neural_cmd(s_Swerve, limelight_Neural, CoralId, CoralDistance))
            );
        // AlignAngle.onTrue(new Align_angle(s_Swerve, limelight).withTimeout(0.7));

        // -------------------
        // copilotJoystick.x().whileTrue(
        //     Commands.parallel(
        //         Commands.run(() -> s_swerve.humanSetPose2Dose())//source pid
        //     )
        // );

        // copilotJoystick.x().onFalse(
        //     Commands.parallel(
        //       Commands.runOnce(() -> intakeSubsystem.angleout(0)),
        //       Commands.runOnce(() -> intakeSubsystem.stop())
        //     )
        // );

        // copilotJoystick.y().whileTrue(
        //     Commands.parallel(
        //         Commands.run(() -> intakeSubsystem.setangle(-1.7)),
        //         Commands.run(() -> intakeSubsystem.algaesuck())
        //         //shoot pid                Commands.run(() -> intakeSubsystem.algaeshoot())
        //     )
        // );

        // copilotJoystick.y().whileFalse(
        //     Commands.parallel(
        //       Commands.run(() -> intakeSubsystem.setangle(0)),
        //       Commands.run(() -> intakeSubsystem.suckstop())
        //     )
        // );
        // copilotJoystick.y().whileTrue(Commands.run(() -> shooterSubsystem.angleout(1)));
        // copilotJoystick.y().onFalse(Commands.runOnce(() -> shooterSubsystem.angleout(0)));
        copilotJoystick.b().whileTrue(Commands.parallel(
            Commands.runOnce(() -> intakeSubsystem.resetencoder())
          ));
    
        // copilotJoystick.rightTrigger().whileTrue(Commands.run(() -> intakeSubsystem.algaeshoot()));//intake go
        // copilotJoystick.rightTrigger().onFalse(new ParallelCommandGroup(
        //     Commands.run(() -> intakeSubsystem.suckstop())
        // ));

        // copilotJoystick.start().onTrue(new Auto_coral_cmd(elevatorSubsystem,shooterSubsystem,3));
        // copilotJoystick.start().onFalse(Commands.runOnce(() -> Auto_coral_cmd.finish()));
        

        
        // copilotJoystick.leftTrigger().whileTrue(
        //     new ParallelCommandGroup(
        //         Commands.run(() -> intakeSubsystem.setangle(-1.59)),
        //         Commands.run(() -> intakeSubsystem.algaesuck())
        //     )
        // );//intake pid (suck)
        // copilotJoystick.leftTrigger().onFalse(Commands.run(() -> intakeSubsystem.suckstop()));//output = 0

        copilotJoystick.leftBumper().whileTrue(Commands.run(() -> shooterSubsystem.coralSuck()));
        copilotJoystick.leftBumper().onFalse(Commands.runOnce(() -> shooterSubsystem.stop()));
        copilotJoystick.rightBumper().whileTrue(Commands.run(() -> shooterSubsystem.coralshoot()).withTimeout(0.5));
        copilotJoystick.rightBumper().onFalse(Commands.runOnce(() ->shooterSubsystem.setshooter(0)));
        // copilotJoystick.rightBumper().whileTrue(Commands.run(() -> shooterSubsystem.coralshoot()));

    

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new PathPlannerAuto("ddd");
      }
}