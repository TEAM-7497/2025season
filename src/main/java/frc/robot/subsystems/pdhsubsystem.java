package frc.robot.subsystems;
// import com.revrobotics.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class pdhsubsystem extends SubsystemBase { 
    public final PowerDistribution pdh = new PowerDistribution(1 , ModuleType.kRev);
    public pdhsubsystem(){
        pdh.setSwitchableChannel(true);
    }
}
