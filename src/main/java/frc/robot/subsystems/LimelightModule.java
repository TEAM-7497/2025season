
package frc.robot.subsystems;

import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.LimelightHelpers.LimelightTarget_Detector;
import edu.wpi.first.networktables.NetworkTableInstance;
// import frc.robot.subsystems.LimelightHelpers;
public class LimelightModule {

    public static enum LightMode {
        Current(0),
        Off(1),
        Blink(2),
        On(3);

        public final int value;
        
        LightMode(int value) { this.value = value; }
    }

    public static enum camMode {
        Vision_processor(0),
        Driver_Camera(1);

        public final int value;
        
        camMode(int value) { this.value = value; }
    }
    
    public void setLightMode(LightMode mode) {
        NetworkTableInstance.getDefault().getTable(
            "limelight").getEntry("ledMode").setNumber(mode.value);
    }

    /** 
     *  @param tv   Whether the limelight has any valid targets (0 or 1)
     *  @param tx   Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
     *  @param ty   Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
     *  @param ta   Target Area (0% of image to 100% of image)
     *  @param tl   The pipeline’s latency contribution (ms) Add at least 11ms for image capture latency.
     *  @param tshort   Sidelength of shortest side of the fitted bounding box (pixels)
     *  @param tlong    Sidelength of longest side of the fitted bounding box (pixels)
     *  @param thor Horizontal sidelength of the rough bounding box (0 - 320 pixels)
     *  @param tvert    Vertical sidelength of the rough bounding box (0 - 320 pixels)
     *  @param getpipe  True active pipeline index of the camera (0 .. 9)
     *  @param json Full JSON dump of targeting results
     *  @param tclass   Class ID of primary neural detector result or neural classifier result
     */
        public static double getBasicData(String type)
        {
            return NetworkTableInstance.getDefault().getTable("limelight-chengon").getEntry(type).getDouble(0);
        }

    /** 
     *  @param botpose  Robot transform in field-space. Translation (X,Y,Z) Rotation(X,Y,Z)
     *  @param botpose_wpiblue  Robot transform in field-space (blue driverstation WPILIB origin). Translation (X,Y,Z) Rotation(X,Y,Z)
     *  @param botpose_wpired   Robot transform in field-space (red driverstation WPILIB origin). Translation (X,Y,Z) Rotation(X,Y,Z)
     *  @param camerapose_targetspace   3D transform of the camera in the coordinate system of the primary in-view AprilTag (array (6))
     *  @param targetpose_cameraspace   3D transform of the primary in-view AprilTag in the coordinate system of the Camera (array (6))
     *  @param targetpose_robotspace    3D transform of the primary in-view AprilTag in the coordinate system of the Robot (array (6))
     *  @param botpose_targetspace  3D transform of the robot in the coordinate system of the primary in-view AprilTag (array (6))
     *  @param tid  ID of the primary in-view AprilTag
     */
    public double[] get_AprilTag_3D_Data(String type)
    { 
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry(type).getDoubleArray(new double[10]);
    }

    /** 
     *  @param ledMode  Sets limelight’s LED state
     *  @param camMode  Sets limelight’s operation mode
     *  @param pipeline Sets limelight’s current pipeline
     *  @param stream   Sets limelight’s streaming mode
     *  @param snapshot Allows users to take snapshots during a match
     *  @param crop Sets the crop rectangle. The pipeline must utilize the default crop rectangle in the web interface. The array must have exactly 4 entries.
     */
    public void set_Cam_Control(String type, int value)
    {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry(type).setNumber(value);
    }

    public LimelightTarget_Detector[] getNeuralNetworkData() {
        LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults("limelight");
        return results.targets_Detector != null ? results.targets_Detector : new LimelightTarget_Detector[0];
        // return results.targets_Detector;

    }
}