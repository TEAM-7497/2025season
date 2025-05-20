// package frc.robot.subsystems;


// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.DutyCycle;
// import edu.wpi.first.wpilibj.motorcontrol.Talon;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;




// import com.ctre.phoenix6.StatusSignal;
// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.ctre.phoenix6.controls.DutyCycleOut;
// import com.ctre.phoenix6.controls.NeutralOut;
// import com.ctre.phoenix6.controls.PositionVoltage;
// import com.ctre.phoenix6.controls.VoltageOut;

// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// // 2*falcon

// public class ClimberSubsystem extends SubsystemBase {

//     public TalonFX climber = new TalonFX(25); 
    
//     final PositionVoltage m_position = new PositionVoltage(0);
//     public DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    
//   //-------------------------monkey
//   public VoltageOut voltageOut = new VoltageOut(0.0);
//   final VoltageOut m_request = new VoltageOut(12);
  
//   //--------------------------
//   /** Creates a new ElevatorSubsystem. */
//   public ClimberSubsystem() {
//     var slot0Configs = new Slot0Configs();
//     // slot0Configs.kV = 0.12;
//     // slot0Configs.kP = 0.11;
//     // slot0Configs.kI = 0.48;
//     // slot0Configs.kD = 0.01;
//     slot0Configs.kV = 0.12;
//     slot0Configs.kP = 0.32;
//     slot0Configs.kI = 0.48;
//     slot0Configs.kD = 0.01;
    
//     climber.setNeutralMode(NeutralModeValue.Brake);
   
//     climber.getConfigurator().apply(slot0Configs, 0.050);
  

//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }

//   @Override
//   public void simulationPeriodic() {
//     // This method will be called once per scheduler run during simulation
//   }

// //   public double getPosition() {
// //     var rotorPosSignal = climberL.getRotorPosition();
// //     return rotorPosSignal.getValueAsDouble();
// // }//圈數

// // public void set(double dou){
// //   m_position.withPosition(dou);
// //   climberL.setControl(m_position);
// //   climberR.setControl(m_position);
// //   SmartDashboard.putNumber("encodervalue",getPosition());
// // }


//   public void up() {
//     // elevatorl.setControl(dutyCycleOut.withOutput(0.25));
//     // elevatorr.setControl(dutyCycleOut.withOutput(0.25));
//     climber.setControl(voltageOut.withOutput(-6));
    
//     // SmartDashboard.putNumber("encodervalue",getPosition());
//   }
//   public void  down() {
//     // elevatorl.setControl(dutyCycleOut.withOutput(-0.25));
//     // elevatorr.setControl(dutyCycleOut.withOutput(-0.25));
//     climber.setControl(voltageOut.withOutput(6));
   
//   }
//   public void stop() {
//     // elevatorl.setControl(dutyCycleOut.withOutput(0));
//     // elevatorr.setControl(dutyCycleOut.withOutput(0 ));
//     climber.setControl(voltageOut.withOutput(0));
  

   
//   }

  



// //   public void getleft() {
// //   getleft_motor.setControl(up.withOutput(-0.1));
// // }
// //   public void getright() {
// //   getleft_motor.setControl(up.withOutput(0.1));
// // }
// //   public void stop_getleft() {
// //   getleft_motor.setControl(up.withOutput(0));
// //   }
// } 
