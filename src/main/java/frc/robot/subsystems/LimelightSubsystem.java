package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.utils.APTree;
import frc.robot.utils.Pose;

public class LimelightSubsystem extends SubsystemBase {

  private static NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight-aptag");
  private static APTree tagToDistanceLookup = new APTree();
  private static APTree distanceToSpeedLookup = new APTree();
  private static double distanceMeters = 0;
  private static final double CAMERA_TO_CENTRE = 0.349;

  public double getDistanceMeters() {
    return distanceMeters;
  }

  // -------------------------------------------------------
  //   Per-tag TY → Distance tables
  // -------------------------------------------------------
  private static final double[][] T10_DISTANCE_DATA = {
    {21.10, 0.93},
    {14.81, 1.133},
    {11.56, 1.59},
    {8.65,  1.85},
    {5.84,  2.157},
    {4.29,  2.357},
  };

  private static final double[][] T2_DISTANCE_DATA = {
    {0, 2.03},
    {-2.74, 2.45},
    {-5.11, 3.31},
  };

  private static final double[][] T5_DISTANCE_DATA = {
    {21.10, 10.93},
    {14.81, 11.133},
    {11.56, 11.59},
    {8.65,  11.85},
    {5.84,  12.157},
    {4.29,  12.357},
  };

  private static final double[][] T13_DISTANCE_DATA = {
    {7.45, 0.296},
    {1.51, 0.545},
    {-1.39, 0.734},
    {-3.51, 0.945},
    {-4.56, 1.126},
    {-5.70, 1.306},
    {-7.03, 1.522},
    {-8.29, 1.685},
    {-9.32, 1.898},
    {-10.03, 2.127},
    {-10.59, 2.395},
    {-10.90, 2.582},
    {-11.17, 2.800},
    {-11.56, 2.950},
    {-11.71, 3.228},
  };

  private static final double[][] T1_DISTANCE_DATA = { // Official
    {0.63, 0.590},
    {-3.16, 0.843},
    {-4.20, 0.940},
    {-5.34, 1.08},
    {-5.70, 1.12},
    {-5.99, 1.157},
    {-6.34, 1.205},
    {-6.74, 1.278},
    {-7.05, 1.332},
    {-7.77, 1.480},
    {-8.56, 1.666},
    {-9.08, 1.830},
    {-9.53, 1.912},
    {-9.97, 2.040},
    {-10.54, 2.180},
    {-11.05, 2.361},
    {-11.54, 2.592},
    {-12.00, 2.866},
    {-12.19, 3.000},
  };

  private static final double[][] T12_DISTANCE_DATA = { // Official
    {0.52, 0.535},
    {-2.00, 0.706},
    {-3.57, 0.838},
    {-4.83, 0.986},
    {-5.69, 1.094},
    {-6.58, 1.257},
  };

  // Distance → Shooter RPS
  private static final double[][] SPEED_DATA = {
    {2.03,  62.5},
    {2.45, 64},
    {3.31, 71},
  };

  public LimelightSubsystem() {
    distanceToSpeedLookup.InsertValues(SPEED_DATA);
  }

  // -------------------------------------------------------
  //   Tag Classification
  // -------------------------------------------------------

  /** Returns the TY→Distance table for a given tag ID, or null if unsupported. */
  public double[][] getDataForTag(double IDNum) {
    switch ((int) Math.round(IDNum)) {
      case 10: return T10_DISTANCE_DATA;
      case 2:  return T2_DISTANCE_DATA;
      case 5:  return T5_DISTANCE_DATA;
      case 13: return T13_DISTANCE_DATA;
      case 1:  return T1_DISTANCE_DATA;
      case 12: return T12_DISTANCE_DATA;
      default: return null;
    }
  }

  /** Tags that are valid targets for auto-aiming/rotation. */
  public boolean isAimTag(double IDNum) {
    int id = (int) Math.round(IDNum);
    return id == 2 || id == 5 || id == 10 || id == 13 || id == 12 || id == 1;
  }

  // -------------------------------------------------------
  //   Computed Values
  // -------------------------------------------------------

  /** Returns distance to the given tag ID using the current TY reading. */
  public double getTagDistance(double IDNum) {
    double[][] data = getDataForTag(IDNum);
    if (data == null) return 0;

    tagToDistanceLookup = new APTree();
    tagToDistanceLookup.InsertValues(data);
    return tagToDistanceLookup.GetValue(getTY());
  }

  /** Returns the shooter RPS for the given tag ID based on distance. */
  public double getShooterRPS(double IDNum) {
    if (getDataForTag(IDNum) == null) return 30.0;
    return distanceToSpeedLookup.GetValue(getTagDistance(IDNum));
  }

  /** Returns TX if the robot needs to turn, 0 if already aligned or no target. */
  public double getTurnAngle() {
    if (!hasValidTarget()) return 0;
    if (Math.abs(getTX()) <= VisionConstants.kAngleTolerance) return 0;
    return getTX();
  }

  // -------------------------------------------------------
  //   Raw Limelight Network Table Accessors
  // -------------------------------------------------------

  public double getDoubleEntry(String entry) {
    return limelight.getEntry(entry).getDouble(0);
  }

  public double[] getArraryEntry(String entry) {
    return limelight.getEntry(entry).getDoubleArray(new double[6]);
  }

  public boolean hasValidTarget() {
    return limelight.getEntry("tv").getDouble(0) == 1;
  }

  public double getTID() {
    return getDoubleEntry("tid");
  }

  public double getTA()  {
    return getDoubleEntry("ta");
  }
  public double getTX()  {
    return getDoubleEntry("tx");
  }
  public double getTY()  {
    return getDoubleEntry("ty");
  }

  public Pose getPoseFromTag(double robotYawDeg) {

    if (!hasValidTarget()) {
        return null; // or return current pose instead
    }

    double distance = distanceMeters + CAMERA_TO_CENTRE;  // already updating in periodic
    double tx = getTX();

    // Bearing from field frame
    double bearingRad = Math.toRadians(robotYawDeg - tx);

    // Tag assumed at (0,0)
    double robotX = -distance * Math.cos(bearingRad);
    double robotY = -distance * Math.sin(bearingRad);

    return new Pose(robotX, robotY, robotYawDeg);
}

  // -------------------------------------------------------
  //   Periodic
  // -------------------------------------------------------

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("HAS TARGET", hasValidTarget());

    if (hasValidTarget()) {
      double tid = getTID();
      distanceMeters = getTagDistance(tid);
      SmartDashboard.putNumber("tid", tid);
      SmartDashboard.putNumber("ta", getTA());
      SmartDashboard.putNumber("ty", getTY());
      SmartDashboard.putNumber("tx", getTX());
      SmartDashboard.putNumber("Distance from tag", getTagDistance(tid));
      SmartDashboard.putBoolean("Is Aim Tag", isAimTag(tid));
    }
  }
}