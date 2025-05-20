package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


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
// 2*falcon

public class ElevatorSubsystem extends SubsystemBase {

    public TalonFX shooterLeft = new TalonFX(21); 
    public TalonFX shooterRight = new TalonFX(40); 
    public TalonFX Xangle = new TalonFX(42);
    public TalonFX Yangle = new TalonFX(42);
    public TalonFX elevatorr = new TalonFX(1);
    public TalonFX elevatorl = new TalonFX(2);
    final PositionVoltage m_position = new PositionVoltage(0);
    public DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    
  //-------------------------monkey
  public VoltageOut voltageOut = new VoltageOut(0.0);
  final VoltageOut m_request = new VoltageOut(12);
  
  //--------------------------
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    var slot0Configs = new Slot0Configs();
    slot0Configs.kV = 0.12;
    slot0Configs.kP = 0.5;
    slot0Configs.kI = 0.18;
    slot0Configs.kD = 0.01;
    // setPosition(0);
    
    elevatorr.setNeutralMode(NeutralModeValue.Brake);
    elevatorl.setNeutralMode(NeutralModeValue.Brake);
    elevatorl.getConfigurator().apply(slot0Configs, 0.050);
    elevatorr.getConfigurator().apply(slot0Configs, 0.050);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("elevatorencoder " , elevatorl.getPosition().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double getPosition() {
    var rotorPosSignal = elevatorl.getRotorPosition();
    return -rotorPosSignal.getValueAsDouble();
  }//圈數

// public void setPosition(double dou) {
// elevatorl.setPosition(dou);
// elevatorr.setPosition(dou);
// }

public void resetencoder(){
  elevatorl.setPosition(0);
  elevatorr.setPosition(0);
}

// public void set(double dou){
//   elevatorl.setControl(voltageOut.withOutput(-dou));
//   elevatorr.setControl(voltageOut.withOutput(-dou));

// }

public void show(){
  SmartDashboard.putNumber("eleelelelelele",getPosition());
}


  public void shoot() {
   
    shooterLeft.setControl(voltageOut.withOutput(-1.5));
    shooterRight.setControl(voltageOut.withOutput(-1.5));
  }
  public void up(){
    // Yangle.setControl(voltageOut.withOutput(1));
  }
  public void down() {
    // elevatorl.setControl(dutyCycleOut.withOutput(-0.25));
    // elevatorr.setControl(dutyCycleOut.withOutput(-0.25));
    Yangle.setControl(voltageOut.withOutput(-1));
  }
  public void TurnLeft(){
    Xangle.setControl(voltageOut.withOutput(1));
  }

   public void TurnRight(){
    Xangle.setControl(voltageOut.withOutput(-1));
  }
  // public void slowdown() {
  //   // elevatorl.setControl(dutyCycleOut.withOutput(-0.25));
  //   // elevatorr.setControl(dutyCycleOut.withOutput(-0.25));
  //   elevatorl.setControl(voltageOut.withOutput(0.7));
  //   elevatorr.setControl(voltageOut.withOutput(0.7));
  // }
  public void slowdown(){

  }
  public void set(double output){

  }
  public void stop() {
    Xangle.setControl(voltageOut.withOutput(0));
    Yangle.setControl(voltageOut.withOutput(0));
  }

  public void stopShoot(){
    shooterLeft.setControl(voltageOut.withOutput(0));
    shooterRight.setControl(voltageOut.withOutput(0));
  }

  // public void zero(){
  //   elevatorl.setControl(voltageOut.withOutput(0));
  //   elevatorr.setControl(voltageOut.withOutput(0));
  // }
 


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
