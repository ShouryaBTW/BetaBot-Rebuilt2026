package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.utils.APPID;
import frc.robot.utils.Calculations;
import frc.robot.utils.Pose;

public class TurnToAngle extends Command {
  private final DriveSubsystem driveSubsystem;
  private final LimelightSubsystem limelightSubsystem; // null if hardcoded
  private final APPID turnPID;

  private double targetAngleDeg;
  private final double angleToleranceDeg;

  private static final double kTurnP = 0.02;
  private static final double kTurnI = 0.0;
  private static final double kTurnD = 0.0;
  private static final double kMaxRot = 0.25;

  // Hardcoded angle
  public TurnToAngle(DriveSubsystem driveSubsystem, double targetAngleDeg, double angleToleranceDeg) {
    this.driveSubsystem = driveSubsystem;
    this.limelightSubsystem = null;
    this.targetAngleDeg = targetAngleDeg;
    this.angleToleranceDeg = angleToleranceDeg;

    this.turnPID = new APPID(kTurnP, kTurnI, kTurnD, angleToleranceDeg);
    this.turnPID.setMaxOutput(kMaxRot);

    addRequirements(driveSubsystem);
  }

  public TurnToAngle(DriveSubsystem driveSubsystem, double targetAngleDeg) {
    this(driveSubsystem, targetAngleDeg, 2.0);
  }

  // Limelight mode — dynamically tracks tx each loop
  public TurnToAngle(DriveSubsystem driveSubsystem, LimelightSubsystem limelightSubsystem, double angleToleranceDeg) {
    this.driveSubsystem = driveSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    this.targetAngleDeg = 0;
    this.angleToleranceDeg = angleToleranceDeg;

    this.turnPID = new APPID(kTurnP, kTurnI, kTurnD, angleToleranceDeg);
    this.turnPID.setMaxOutput(kMaxRot);

    addRequirements(driveSubsystem);
  }

  public TurnToAngle(DriveSubsystem driveSubsystem, LimelightSubsystem limelightSubsystem) {
    this(driveSubsystem, limelightSubsystem, 2.0);
  }

  @Override
  public void initialize() {
    turnPID.reset();
  }

  @Override
  public void execute() {
    if (limelightSubsystem != null) {
      if (!limelightSubsystem.hasValidTarget()) {
        driveSubsystem.drive(0, 0, 0, true);
        return;
      }

      // FIX 1: Instead of converting tx into a field-space heading (which created a
      // moving target that the robot could never settle on), we now use tx directly
      // as the error since it already represents how far off-center the tag is.
      double tx = limelightSubsystem.getTX();
      turnPID.setDesiredValue(0);
      double rotCmd = turnPID.calculate(tx); // Flip sign here if robot turns the wrong way
      driveSubsystem.drive(0.0, 0.0, rotCmd, true);
      return;
    }

    // Hardcoded angle path (unchanged)
    Pose currentPose = driveSubsystem.getPose();
    double currentAngle = Calculations.normalizeAngle360(currentPose.getAngle());
    double targetAngle  = Calculations.normalizeAngle360(targetAngleDeg);
    double angleError   = Calculations.shortestAngularDistance(targetAngle, currentAngle);

    turnPID.setDesiredValue(0);
    double rotCmd = turnPID.calculate(-angleError);
    driveSubsystem.drive(0.0, 0.0, rotCmd, true);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0.0, 0.0, 0.0, true);
  }

@Override
public boolean isFinished() {
  if (limelightSubsystem != null) {
    if (!limelightSubsystem.hasValidTarget()) return false;
    return Math.abs(limelightSubsystem.getTX()) <= angleToleranceDeg;
  }

  Pose currentPose = driveSubsystem.getPose();
  double currentAngle = Calculations.normalizeAngle360(currentPose.getAngle());
  double targetAngle  = Calculations.normalizeAngle360(targetAngleDeg);
  double angleError   = Math.abs(Calculations.shortestAngularDistance(targetAngle, currentAngle));
  return angleError <= angleToleranceDeg;
  }
}
