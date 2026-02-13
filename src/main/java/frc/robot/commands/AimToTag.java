package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * Command to aim the robot at an AprilTag using Limelight.
 * 
 * Strategy:
 * - Reads tx from Limelight (horizontal offset in degrees)
 * - Calculates target angle = current angle + tx
 * - Uses TurnToAngle command to do the actual rotation
 * 
 * This is a "one-shot" command - it reads tx once and turns.
 * For continuous tracking, you'd need to re-run this command.
 */
public class AimToTag extends Command {

  private final DriveSubsystem driveSubsystem;
  private final LimelightSubsystem limelightSubsystem;

 // Holds the TurnToAngle command
  private Command turnCommand;  

  public AimToTag(DriveSubsystem driveSubsystem, LimelightSubsystem limelightSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.limelightSubsystem = limelightSubsystem;


    addRequirements(driveSubsystem, limelightSubsystem);
  }


  private static final double kAngleTolerance = 2.0;  // degrees

@Override
public void execute() {
    if (!limelightSubsystem.hasValidTarget()) {
        return;
    }

    double tx = limelightSubsystem.getTX();
    if (Math.abs(tx) <= VisionConstants.kAngleTolerance) {
    return;
}

    // Don't turn if already close enough!
    if (Math.abs(tx) <= kAngleTolerance) {
        return;
    }

    double currentAngle = driveSubsystem.getHeading();
    double targetAngle = currentAngle - tx;
    
    turnCommand = new TurnToAngle(driveSubsystem, targetAngle);
    turnCommand.schedule();
}
  
  @Override
  public boolean isFinished() {
    // End if no target was found
    if (!limelightSubsystem.hasValidTarget()) {
      return true;
    }
    
    // End when TurnToAngle command finishes
    return turnCommand != null && !turnCommand.isScheduled();
  }
  
  
  @Override
  public void end(boolean interrupted) {
    if (turnCommand != null && turnCommand.isScheduled()) {
    turnCommand.cancel();
    }
  }
}