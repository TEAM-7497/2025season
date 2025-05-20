package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public interface VisionIO {
  // 輸入資料結構，不包含任何自動記錄或 Logger 相關註解
  public static class VisionIOInputs {
    public boolean connected = false;
    public TargetObservation latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public FiducialsObservation latestFiducialsObservations = new FiducialsObservation(0, 0, 0, 0, 0, 0, 0);
    public int[] tagIds = new int[0];
    public Pose3d poseToTag = new Pose3d();
    public double distanceToTag = 0.0;
  }

  /** 簡單目標的角度資訊，不用於姿態估計 */
  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  /** 機器人姿態估計的樣本 */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type) {}

  /** Fiducials 觀測資料 */
  public static record FiducialsObservation(
      int id,
      double txnc,
      double tync,
      double ta,
      double distToCamera,
      double distToRobot,
      double ambiguity) {}

  public static enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2,
    PHOTONVISION
  }

  // 預設不做更新，具體實作在各自類別中實現
  public default void updateInputs(VisionIOInputs inputs) {}
}
