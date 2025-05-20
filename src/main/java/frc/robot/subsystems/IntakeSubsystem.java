package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.ejml.dense.block.InnerRankUpdate_DDRB;

// import com.ctre.phoenix.sensors.CANCoderConfiguration;
// import com.ctre.phoenix.sensors.WPI_CANCoder;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
// import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;



public class IntakeSubsystem extends SubsystemBase {


  private RelativeEncoder encoder;
    

    private SparkMax intakesuck = new SparkMax(2 
      , MotorType.kBrushless);
    private SparkMax intakerangle = new SparkMax(15, MotorType.kBrushless);


    private RelativeEncoder encoderangle = intakerangle.getEncoder();
    
    private SparkClosedLoopController closedLoopController = intakerangle.getClosedLoopController();
    private SparkMaxConfig motorConfig = new SparkMaxConfig();

    // final PositionVoltage m_position = new PositionVoltage(0);
    // public DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    
    
  //-------------------------monkey
  // public VoltageOut voltageOut = new VoltageOut(0.0);
  // final VoltageOut m_request = new VoltageOut(12);
  
  //--------------------------
  /** Creates a new ElevatorSubsystem. */
  public IntakeSubsystem() {

    motorConfig.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1);

    motorConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    // Set PID values for position control. We don't need to pass a closed loop
    // slot, as it will default to slot 0.
    .p(0.21)
    .i(0)
    .d(0)
    .outputRange(-2, 2)
    // Set PID values for velocity control in slot 1
    .p(0.2, ClosedLoopSlot.kSlot1)
    .i(0, ClosedLoopSlot.kSlot1)
    .d(0, ClosedLoopSlot.kSlot1)
    .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
    .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    intakerangle.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void showencoder(){
    SmartDashboard.putNumber("angleencoder", encoderangle.getPosition());
  }

  public double getEncoder(){
    return encoderangle.getPosition();
  }

  public void algaesuck(){
    intakesuck.setVoltage(-7);
  }
  public void algaeshoot(){
    intakesuck.setVoltage(-6);
  }
  public void resetencoder(){
    encoderangle.setPosition(0);
  }

  public void setangle(double agnle){
    closedLoopController.setReference(agnle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }
  public void anglestop(){
    intakerangle.setVoltage(0);
  }
  public void angleout(double output){
    intakerangle.setVoltage(output);
  }
  public void suckstop(){
    intakesuck.setVoltage(0);
    // intakerangle.setVoltage(0.5);
  }









} 
