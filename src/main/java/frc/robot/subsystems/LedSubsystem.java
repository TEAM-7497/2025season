package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.LEDPattern;



import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
// import com.ctre.phoenix.sensors.CANCoderConfiguration;
// import com.ctre.phoenix.sensors.WPI_CANCoder;

// import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.units.measure.Angle;


import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

public class LedSubsystem extends SubsystemBase {
  private final CANdle candle = new CANdle(30);
  private final ColorFlowAnimation shootingMode = new ColorFlowAnimation(0, 255, 0);
  private final RainbowAnimation normal = new RainbowAnimation(0.7 , 0.5 , 20);
  private final StrobeAnimation shooting2 = new StrobeAnimation(255,  0, 0);
  public LedSubsystem() {
  
    candle.configBrightnessScalar(0.5);
    candle.configVBatOutput(VBatOutputMode.Off);
    candle.setLEDs(255, 255, 255);

       
   
  }

  public void shootingLight(){
    candle.animate(shootingMode);
  }

  public void normalLight(){
    candle.animate(normal);
  }

  @Override
  public void periodic() {
   
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  
}
 







