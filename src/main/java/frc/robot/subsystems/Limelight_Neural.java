
package frc.robot.subsystems;
import frc.robot.subsystems.LimelightHelpers.LimelightTarget_Detector;
import frc.robot.subsystems.LimelightHelpers;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.limelight_pipeline;
import frc.robot.subsystems.LimelightModule;
import frc.robot.subsystems.LimelightModule.LightMode;
import frc.robot.subsystems.LimelightModule.camMode;


public class Limelight_Neural extends SubsystemBase {

    // LimelightModule module = new LimelightModule();

    // NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-chengon");

    private final LimelightModule module;
    private LimelightTarget_Detector[] neuralTargets = new LimelightTarget_Detector[0]; 

  
    public Limelight_Neural(LimelightModule module) {
        this.module = module;
        module.set_Cam_Control("pipeline", 0);
    }

    public LimelightTarget_Detector[] getNeuralTargets() {
        return neuralTargets;
    }

    public void setLightMode(LimelightModule.LightMode mode) {
        module.setLightMode(mode);
    }
    

    public LimelightTarget_Detector getBestTarget() {
        if (neuralTargets.length == 0) return null;
        LimelightTarget_Detector best = neuralTargets[0];
        for (LimelightTarget_Detector target : neuralTargets) {
            if (target.confidence > best.confidence) {
                best = target;
            }
        }
        return best;
    }

    public boolean hasValidTarget() {
        return getBestTarget() != null;
    }
    
    public double getTx() {
        // LimelightTarget_Detector[] targets = module.getNeuralNetworkData();
        LimelightTarget_Detector targets = getBestTarget();
        return targets != null ? targets.tx : 0.0;
    }

    public double getTy() {
        LimelightTarget_Detector targets = getBestTarget();
        return targets != null ? targets.ty : 0.0;
    }

    public double getTa() {
        LimelightTarget_Detector targets = getBestTarget();
        return targets != null ? targets.ta : 0.0;
    }

    public String getName(){
        LimelightTarget_Detector targets = getBestTarget();
        return targets != null  ? targets.className : "None";
    }

    public double getId(){
        LimelightTarget_Detector targets = getBestTarget();
        return targets != null ? targets.classID : -1.0;
    }

    public double getBasicData(String type) {
        return LimelightModule.getBasicData(type);
    }
    @Override
    public void periodic()
    {
        neuralTargets = module.getNeuralNetworkData();
        LimelightTarget_Detector bestTarget = getBestTarget();

        if (bestTarget != null) {
            SmartDashboard.putNumber("Neural_tx", neuralTargets[0].tx);
            SmartDashboard.putNumber("Neural_ty", neuralTargets[0].ty);
            SmartDashboard.putNumber("Neural_ta", neuralTargets[0].ta);
            SmartDashboard.putString("Neural_class", neuralTargets[0].className);
            SmartDashboard.putNumber("conf", neuralTargets[0].confidence);
        } else {
            SmartDashboard.putString("Neural_status", "No targets");
            SmartDashboard.putNumber("Neural_tx", 0);
            SmartDashboard.putNumber("Neural_ty", 0);
            SmartDashboard.putNumber("Neural_ta", 0);
            SmartDashboard.putString("Neural_class", "none");
            SmartDashboard.putNumber("conf", 0);
        }
        
    }
    
}
