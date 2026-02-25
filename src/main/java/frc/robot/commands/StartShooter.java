package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class StartShooter extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final LimelightSubsystem limelightSubsystem; // null if hardcoded
  private final double targetRPS; // ignored if using limelight
  private double limeRPS;

  // Hardcoded RPS
  public StartShooter(ShooterSubsystem shooterSubsystem, double targetRPS) {
    this.shooterSubsystem = shooterSubsystem;
    this.limelightSubsystem = null;
    this.targetRPS = targetRPS;

    addRequirements(shooterSubsystem);
  }

  // Limelight-calculated RPS
  public StartShooter(ShooterSubsystem shooterSubsystem, LimelightSubsystem limelightSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    this.targetRPS = 0; // unused

    addRequirements(shooterSubsystem); // no limelight requirement, just reading it
  }

  @Override
  public void initialize() {
    if (limelightSubsystem != null) {
      limeRPS = limelightSubsystem.getShooterRPS(limelightSubsystem.getTID());
    }
  }

  @Override
  public void execute() {
    if (limelightSubsystem != null) {
      shooterSubsystem.setVelocity(limeRPS);
    } else {
      shooterSubsystem.setVelocity(targetRPS);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}