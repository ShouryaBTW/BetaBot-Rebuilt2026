package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.utils.APTree;
import frc.robot.utils.Pose;

public class LimelightSubsystem extends SubsystemBase {

  private static NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
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
    {19.06, 0.71},
    {13.98, 0.93},
    {11.34, 1.00},
    {6.71, 1.27},
    {4.12, 1.42},
    {1.17, 1.67},
    {-0.05, 1.79},
    {-1.25, 1.92},
    {-2.35, 2.10},
    {-3.38, 2.20},
    {-4.40, 2.40},
    {-5.01, 2.58},
  };

  private static final double[][] T2_DISTANCE_DATA = {
    {4.93 ,1.30},
    {4.15, 1.40},
    {2.37, 1.52},
    {0.20, 1.73},
    {-1.86, 1.93},
    {-3.60, 2.20},
    {-4.97, 2.50},
    {-6.00, 2.74},
    {-6.51, 2.90},
    {-7.22, 3.07},
  };

  private static final double[][] T5_DISTANCE_DATA = {
    {21.10, 10.93},
  };

  private static final double[][] T13_DISTANCE_DATA = {
    {-15.91, 0.70},
    {-16.53, 0.94},
    {-16.71, 1.10},
    {-16.88, 1.23},
    {-17.00, 1.42},
    {-17.18, 1.64},
    {-17.29, 2.00},
    {-17.31, 2.30},
    {-17.36, 2.53},
    {-17.38, 2.90},
    {-17.43, 3.00},
  };

  private static final double[][] T1_DISTANCE_DATA = { // Official
    {14,98, 0.52},
    {4.87, 0.81},
    {2.10, 0.94},
    {-1.02, 1.20},
    {-4.00, 1.40},
    {-6.12, 1.69},
    {-7.77, 1.96},
    {-9.34, 2.35},
    {-10.67, 2.74},
    {-11.62 ,3.00},
    {-12.41, 3.35},
  };

  private static final double[][] T12_DISTANCE_DATA = { // Official
    {16.78, 0.52},
    {7.14, 0.80},
    {2.16, 0.97},
    {-1.41, 1.20},
    {-5.12, 1.54},
    {-7.24, 1.86},
    {-8.72, 2.17},
  };

  // Distance → Shooter RPS
  private static final double[][] SPEED_DATA = {
    {1.67, 56}, // First good point
    {1.79, 58},
    {1.92, 59},
    {2.10, 60},
    {2.20, 62},
    {2.40, 62},
    {2.58, 63},
    {2.90, 67},
    {3.07, 68},
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