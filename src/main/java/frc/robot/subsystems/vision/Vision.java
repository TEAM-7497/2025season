package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.util.GeomUtil;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  // 【修改】：將 VisionIOInputsAutoLogged[] 改為使用 VisionIO.VisionIOInputs[]
  private final VisionIO.VisionIOInputs[] inputs;
  private final Alert[] disconnectedAlerts;

  // 以場上 AprilTag 的 2D 位置建立靜態 Map
  public static final Map<Integer, Pose2d> tagPoses2d = new HashMap<>();

  static {
    for (int i = 1; i <= FieldConstants.aprilTagCount; i++) {
      tagPoses2d.put(
          i,
          FieldConstants.defaultAprilTagType
              .getLayout()
              .getTagPose(i)
              .map(Pose3d::toPose2d)
              .orElse(new Pose2d()));
    }
  }

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    // 【修改】：使用 VisionIO.VisionIOInputs 作為資料結構，不再使用 auto-logged 版本
    this.inputs = new VisionIO.VisionIOInputs[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIO.VisionIOInputs();
    }

    // 為每個視覺來源建立斷線警告
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert("Vision camera " + i + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * 回傳指定攝影機最新目標觀測的水平偏移角
   */
  
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    // 更新所有視覺來源的資料
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
    }

    // 逐個攝影機處理資料
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // 更新斷線狀態
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // 處理每筆機器人姿態觀測資料
      for (var observation : inputs[cameraIndex].poseObservations) {
        boolean rejectPose =
            observation.tagCount() == 0
                || (observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity)
                || Math.abs(observation.pose().getZ()) > maxZError
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth();
        if (rejectPose) {
          continue;
        }

        double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }
consumer.accept(
    observation.pose().toPose2d(),
    observation.timestamp(),
    (Matrix<N3, N1>) VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));

      }
    }
  }

  /**
   * VisionConsumer 接口，供上層接收視覺處理後的位姿資訊
   */
  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  /**
   * 檢查指定攝影機中是否有偵測到特定 tag ID
   */
  public boolean seenTagId(int id, int camera) {
    for (var tagId : inputs[camera].tagIds) {
      if (tagId == id) return true;
    }
    return false;
  }

  /**
   * 回傳指定攝影機計算出的相機到 tag 距離
   */
  public double getDistanceToTag(int camera) {
    return inputs[camera].latestFiducialsObservations.distToCamera();
  }

  /**
   * 目前僅為佔位方法，回傳 false
   */
  public boolean scoreReady() {
    return false;
  }

  /**
   * 方法一：利用視覺數據計算機器人相對於 tag 的 Field Pose
   */
  // public Pose2d getFieldPoseUsingTag(int camera, Rotation2d rotation) {
  //   double xAngle =
  //       inputs[camera].latestTargetObservation.tx().getDegrees() - VisionConstants.cameraAngleOffsetsYaw[camera];
  //   double yAngle =
  //       inputs[camera].latestTargetObservation.ty().getDegrees() - VisionConstants.cameraAngleOffsetsPitch[camera];
  //   double distance2d = getDistanceToTag(camera) * Math.cos(Units.degreesToRadians(yAngle));

  //   double yDist = Math.sin(Units.degreesToRadians(xAngle)) * distance2d;
  //   double xDist = Math.cos(Units.degreesToRadians(xAngle)) * distance2d;

  //   Pose2d robotLocalPose =
  //       new Pose2d(xDist, yDist, rotation)
  //           .transformBy(new Transform2d(-0.280509, 0.257131, new Rotation2d()));
  //   Pose2d aprilTagFieldPose = tagPoses2d.get(inputs[camera].latestFiducialsObservations.id());

  //   return aprilTagFieldPose.transformBy(
  //       new Transform2d(robotLocalPose.getTranslation(),
  //           robotLocalPose.getRotation().rotateBy(Rotation2d.fromDegrees(180))));
  // }

  /**
   * 方法二：利用 3D 座標轉換計算機器人 Field Pose
   */
  public Pose2d getFieldPoseUsingTag2(int camera, Rotation2d rot) {
    double tx = inputs[camera].latestTargetObservation.tx().getRadians();
    double ty = inputs[camera].latestTargetObservation.ty().getRadians();
    Pose2d tagPose2d = tagPoses2d.get(inputs[camera].latestFiducialsObservations.id());
    if (tagPose2d == null) return new Pose2d();
    Pose3d cameraPose = VisionConstants.cameraOffsets[camera];

    Translation2d camToTagTranslation =
        new Pose3d(Translation3d.kZero, new Rotation3d(0, ty, tx))
            .transformBy(new Transform3d(new Translation3d(getDistanceToTag(camera), 0, 0), Rotation3d.kZero))
            .getTranslation()
            .rotateBy(new Rotation3d(0, cameraPose.getRotation().getY(), 0))
            .toTranslation2d();

    Rotation2d camToTagRotation =
        rot.plus(cameraPose.toPose2d().getRotation().plus(camToTagTranslation.getAngle()));

    Translation2d fieldToCameraTranslation =
        new Pose2d(tagPose2d.getTranslation(), camToTagRotation.plus(Rotation2d.kPi))
            .transformBy(GeomUtil.toTransform2d(camToTagTranslation.getNorm(), 0.0))
            .getTranslation();

    Pose2d robotPose =
        new Pose2d(fieldToCameraTranslation, rot.plus(cameraPose.toPose2d().getRotation()))
            .transformBy(new Transform2d(cameraPose.toPose2d(), Pose2d.kZero));

    robotPose = new Pose2d(robotPose.getTranslation(), rot);

    return robotPose;
  }
}
