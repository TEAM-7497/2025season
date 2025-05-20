package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Ultrasonic;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

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
import edu.wpi.first.wpilibj.Timer;




 public class ShooterSubsystem extends SubsystemBase {

  

  
    private static boolean resetonce = false;
    private SparkMax shooter = new SparkMax(16, MotorType.kBrushless);
    public static DigitalInput photoSensor = new DigitalInput(8);
    Timer time = new Timer();
    
    
     
    public boolean getCoral() {
      return !photoSensor.get();
    }
    private RelativeEncoder encoder = shooter.getEncoder();
   
    
   
    private SparkMaxConfig shooterConfig = new SparkMaxConfig();

    // final PositionVoltage m_position = new PositionVoltage(0);
    // public DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    
    
  //-------------------------monkey
  // public VoltageOut voltageOut = new VoltageOut(0.0);
  // final VoltageOut m_request = new VoltageOut(12);
  
  //--------------------------
  /** Creates a new ElevatorSubsystem. */
  public ShooterSubsystem() {
    
    shooterConfig.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1);
   

    shooterConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    // Set PID values for position control. We don't need to pass a closed loop
    // slot, as it will default to slot 0.
    .p(0.07)
    .i(0)
    .d(0)
    .outputRange(-1, 1)
    // Set PID values for velocity control in slot 1
    .p(0.1, ClosedLoopSlot.kSlot1)
    .i(0, ClosedLoopSlot.kSlot1)
    .d(0, ClosedLoopSlot.kSlot1)
    .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
    .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

   

  }

  @Override
  public void periodic() {
    
    // SmartDashboard.putNumber("ultrasonic" ,getDistanceInches());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void showencoder(){
    SmartDashboard.putNumber("shooteranlgeencoder", encoder.getPosition());
  }

  public void setshooter(double outputshoot){
    shooter.setVoltage(outputshoot);
  }

  public void resetencoder(){
    // encoderangle.setPosition(0);
  }

  public void setangle(double agnle){
    // closedLoopController.setReference(agnle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public void stop(){
    shooter.setVoltage(0);
    resetonce = false;
  }

  public void angleout(double output){
    // shooterangle.setVoltage(output);
  }

  // public double getEncoder(){
  //   return encoderangle.getPosition();
  // }

  public void coralshoot(){
    shooter.setVoltage(2);
  }

  public void coralSuck(){
    if(getCoral() && resetonce == false){
      resetonce = true;
      time.reset();
      time.start();
    }
    SmartDashboard.putBoolean("resetonce", resetonce);
    if(getCoral() && time.get()>0.03){
      if(time.get()<=0.37){
        shooter.setVoltage(-2.5);
      }else{
        shooter.setVoltage(0);
        time.stop();
      }
    }else{
      shooter.setVoltage(1.7);
    }
      
  }




















//   public double getPosition() {
//     var rotorPosSignal = elevatorl.getRotorPosition();
//     return rotorPosSignal.getValueAsDouble();
// }//圈數

// public void setPosition(double dou) {
// elevatorl.setPosition(dou);
// elevatorr.setPosition(dou);
// }

// public void set(double dou){
//   elevatorl.setControl(voltageOut.withOutput(-dou));
//   elevatorr.setControl(voltageOut.withOutput(-dou));

// }

// public void show(){
//   SmartDashboard.putNumber("encodervalue",-getPosition());
// }


//   public void up() {
//     // elevatorl.setControl(dutyCycleOut.withOutput(0.25));
//     // elevatorr.setControl(dutyCycleOut.withOutput(0.25));
//     elevatorl.setControl(voltageOut.withOutput(-2));
//     elevatorr.setControl(voltageOut.withOutput(-2));
//   }
//   public void down() {
//     // elevatorl.setControl(dutyCycleOut.withOutput(-0.25));
//     // elevatorr.setControl(dutyCycleOut.withOutput(-0.25));
//     elevatorl.setControl(voltageOut.withOutput(0.4));
//     elevatorr.setControl(voltageOut.withOutput(0.4));

//   }
//   public void stop() {
//     // elevatorl.setControl(dutyCycleOut.withOutput(0));
//     // elevatorr.setControl(dutyCycleOut.withOutput(0 ));
//     elevatorl.setControl(voltageOut.withOutput(-0.4));
//     elevatorr.setControl(voltageOut.withOutput(-0.4));
//   }
//   public void zero(){
//     elevatorl.setControl(voltageOut.withOutput(0));
//     elevatorr.setControl(voltageOut.withOutput(0));
//   }
 


//   public void getleft() {
//   getleft_motor.setControl(up.withOutput(-0.1));
// }
//   public void getright() {
//   getleft_motor.setControl(up.withOutput(0.1));
// }
//   public void stop_getleft() {
//   getleft_motor.setControl(up.withOutput(0));
//   }
} 
