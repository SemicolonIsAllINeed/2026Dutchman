package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.configs.constants.VisionConstants.FRONT_CAMERA_NAME;

public class Vision extends SubsystemBase {

    private static Vision instance;

    private HttpCamera cameraFeed;
    private AprilTagFieldLayout fieldLayout;
    private NetworkTable cameraTable;

    public Vision() {
        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        cameraTable = NetworkTableInstance.getDefault().getTable(FRONT_CAMERA_NAME);      
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Tag ID", getTagId());
        SmartDashboard.putNumber("X Degrees", getTX());
        SmartDashboard.putBoolean("Tag Detected?", hasTag());
    }

    // ---------- Camera / Limelight Methods ----------
    public boolean hasTag() {
        return cameraTable.getEntry("tv").getDouble(0) == 1;
    }

    public int getTagId() {
        return hasTag() ? (int) cameraTable.getEntry("tid").getInteger(-1) : -1;
    }

    public double getTX() {
        return cameraTable.getEntry("tx").getDouble(0);
    }

    public double getTY() {
        return cameraTable.getEntry("ty").getDouble(0);
    }

    public double getTA() {
        return cameraTable.getEntry("ta").getDouble(0);
    }

    public double[] getCameraPosition() {
        return new double[] {
            getTX(),
            getTY(),
            getTA()
        };
    }

    public Pose2d getRobotPosition() {
        double[] poseNums = cameraTable.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[6]);
        return new Pose2d(
            poseNums[0],
            poseNums[1],
            new Rotation2d(Math.toRadians(poseNums[4]))
        );
    }

    public Pose2d getRelativeRobotPosition() {
        double[] poseNums = cameraTable.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
        return new Pose2d(
            poseNums[0],
            poseNums[2],
            new Rotation2d(Math.toRadians(poseNums[4]))
        );
    }

    public Pose3d get3dRobotPosition() {
        double[] poseNums = cameraTable.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[6]);
        return new Pose3d(
            new Translation3d(poseNums[0], poseNums[1], poseNums[2]),
            new Rotation3d(
                Math.toRadians(poseNums[3]),
                Math.toRadians(poseNums[4]),
                Math.toRadians(poseNums[5])
            )
        );
    }

    public Pose2d getFieldTagPose(int tagId) {
        return fieldLayout.getTagPose(tagId).get().toPose2d();
    }

    public void setLEDMode(int mode) {
        cameraTable.getEntry("ledMode").setNumber(mode);
    }

    public void setRobotOrientation(double yaw, double yawRate) {
        double[] orientation = {yaw, yawRate, 0, 0, 0, 0};
        cameraTable.getEntry("robot_orientation_set").setDoubleArray(orientation);
    }
    public double[] getPicturePosition() {    
        return new double[]{0.0, 0.0, 0.0};

    }

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }
}
