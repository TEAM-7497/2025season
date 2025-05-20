package frc.robot.subsystems;

import java.util.concurrent.BlockingDeque;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.limelight_pipeline;
import frc.robot.subsystems.LimelightModule;
import frc.robot.subsystems.LimelightModule.LightMode;
import frc.robot.subsystems.LimelightModule.camMode;

public class LimelightRight extends SubsystemBase{

    LimelightModule module = new LimelightModule();

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-right");

    @Override
    public void periodic()
    {
        double[] data = {
            LimelightModule.getBasicData("tv"),
            LimelightModule.getBasicData("tx"),
            LimelightModule.getBasicData("ty"),
            LimelightModule.getBasicData("ta"),
            LimelightModule.getBasicData("tid"),
            module.get_AprilTag_3D_Data("botpose")[0],
            module.get_AprilTag_3D_Data("botpose")[1],
            module.get_AprilTag_3D_Data("botpose")[2],

        };
        SmartDashboard.putNumber("Limelight/tv", data[0]);
        SmartDashboard.putNumber("Limelight/tx", data[1]);
        SmartDashboard.putNumber("Limelight/ty", data[2]);
        SmartDashboard.putNumber("Limelight/ta", data[3]);
        SmartDashboard.putNumber("Limelight/tid", data[4]);


    }

    /**
     * @return v, x, y, a
     */
    public double[] get_tag_data(limelight_pipeline pipeline){
        module.set_Cam_Control("pipeline", pipeline.value);

        double[] data = {
            LimelightModule.getBasicData("tv"),
            LimelightModule.getBasicData("tx"),
            LimelightModule.getBasicData("ty"),
            LimelightModule.getBasicData("ta"),
            LimelightModule.getBasicData("tid"),
            module.get_AprilTag_3D_Data("targetpose_robotspace")[0],
            module.get_AprilTag_3D_Data("targetpose_robotspace")[1],
            module.get_AprilTag_3D_Data("targetpose_robotspace")[2],

        };

        return data;
    }

     
    // public void setLightCode(LightMode mode){
    //     module.setLightMode(mode);
    // }
    
    public double[] getAprilTag(){
        return module.get_AprilTag_3D_Data("targetpose_robotspace");
    }

    public int tagdect(){
        return (int)LimelightModule.getBasicData("tv");
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(module.get_AprilTag_3D_Data("botpose")[5]);
    }

    public void setDriverMode(){
        module.set_Cam_Control("pipeline", limelight_pipeline.driver.value);
    }

    public int get_pipeline(){
        return (int)LimelightModule.getBasicData("getpipe");
    }

    public void limeretrorefliective(){
        module.set_Cam_Control("retrorefliective", limelight_pipeline.reflective.value);
    }

    public void limeapriltag(){
        module.set_Cam_Control("apriltag", limelight_pipeline.aprilTag.value);
    }

    public double limelighttx(){
        return table.getEntry("tx").getDouble(0.0);
    }
    public double limelightty(){
        return table.getEntry("ty").getDouble(0.0);
    }
    public double limelighttz(){
        return module.get_AprilTag_3D_Data("camerapose_targetspace")[2];
    }
    public boolean limelighttv(){
        return table.getEntry("tv").getDouble(0.0) == 1.0;
    }
    public int getTagID(){
        return (int)module.get_AprilTag_3D_Data("tid")[0];
    }

//     public double[] getTargetPose(int targetId) {
//     // 獲取 Limelight 的最新結果，請確保使用正確的 Limelight 名稱
//     LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults("limelight_chengon");
//     // 檢查是否有有效的 Fiducial（AprilTag）目標
//     if (results.targets_Fiducials != null) {
//         for (LimelightHelpers.LimelightTarget_Fiducial tag : results.targets_Fiducials) {
//             // 使用適當的方式比較 fiducialID，例如轉換為整數後比較
//             if ((int) tag.fiducialID == targetId) {
//                 // 獲取三維姿態資訊
//                 Pose3d pose3d = tag.getTargetPose_RobotSpace();
//                 // 創建數組並填充 x, y, z, roll, pitch, yaw
//                 double[] targetPoseArray = new double[6];
//                 targetPoseArray[0] = pose3d.getTranslation().getX(); // x 座標
//                 targetPoseArray[1] = pose3d.getTranslation().getY(); // y 座標
//                 targetPoseArray[2] = pose3d.getTranslation().getZ(); // z 座標
//                 targetPoseArray[3] = pose3d.getRotation().getX(); // roll 角度
//                 targetPoseArray[4] = pose3d.getRotation().getY(); // pitch 角度
//                 targetPoseArray[5] = pose3d.getRotation().getZ(); // yaw 角度
//                 return targetPoseArray;
//             }
//         }
//     }

//     SmartDashboard.putBoolean("json gettttt", false);

//     return new double[0];
// }
    

 
}