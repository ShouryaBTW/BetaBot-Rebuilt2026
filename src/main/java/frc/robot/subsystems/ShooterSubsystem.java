package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.utils.APTree;

public class ShooterSubsystem extends SubsystemBase {

  // Intialize motors and make top left motor as leader
  private final TalonFX topLeftMotor;
  private final TalonFX bottomLeftMotor;
  private final TalonFX topRightMotor;
  private final TalonFX bottomRightMotor;

  private final VelocityVoltage velocityRequest;

  private final Timer atSpeedTimer = new Timer();
  private static final double REQUIRED_TIME_AT_SPEED = 0.2;

  private double targetRPS = 0;

  public ShooterSubsystem() {
      topLeftMotor = new TalonFX(40);
      bottomLeftMotor = new TalonFX(41);
      topRightMotor = new TalonFX(42);
      bottomRightMotor = new TalonFX(43);

      velocityRequest = new VelocityVoltage(0).withSlot(0);

      topLeftMotor.getConfigurator().apply(Configs.shooterMotor.shooterConfig);
      bottomLeftMotor.getConfigurator().apply(Configs.shooterMotor.shooterConfig); 
      topRightMotor.getConfigurator().apply(Configs.shooterMotor.shooterConfig);
      bottomRightMotor.getConfigurator().apply(Configs.shooterMotor.shooterConfig);

      // Set follower relationship
      bottomLeftMotor.setControl(
        new Follower(topLeftMotor.getDeviceID(), MotorAlignmentValue.Aligned)
      );

      topRightMotor.setControl(
        new Follower(topLeftMotor.getDeviceID(), MotorAlignmentValue.Opposed)
      );

      bottomRightMotor.setControl(
        new Follower(topLeftMotor.getDeviceID(), MotorAlignmentValue.Opposed)
      );
  }

  /**
   * Set shooter velocity in rotations per second
   * @param rps Target velocity in RPS
   */
  public void setVelocity(double rps) {
    targetRPS = rps;
    topLeftMotor.setControl(velocityRequest.withVelocity(rps));
  }

  /** Stop both pair of motors */
  public void stop() {
    targetRPS = 0;
    atSpeedTimer.stop();
    atSpeedTimer.reset();
    
    topLeftMotor.stopMotor();
  }

  /** Get left (leader) motor velocity in RPS */
  public double getShooterSpeed() {
    return topLeftMotor.getVelocity().getValueAsDouble();
  }


  /**
   * Check if shooter is at target speed for required duration
   * @param tolerance Allowed error in RPS
   * @return True if at speed for REQUIRED_TIME_AT_SPEED seconds
   */
  public boolean atTargetSpeed(double tolerance) {
    // Only check leader motor since follower mirrors it
    boolean withinTolerance = (Math.abs(getShooterSpeed() - targetRPS) < tolerance);

    if (withinTolerance) {
      if (!atSpeedTimer.isRunning()) {
        atSpeedTimer.restart();
      }
    } else {
      atSpeedTimer.stop();
      atSpeedTimer.reset();
    }

    return atSpeedTimer.hasElapsed(REQUIRED_TIME_AT_SPEED);
  }

  /** Get current target velocity in RPS */
  public double getTargetRPS() {
    return targetRPS;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/Target RPS", targetRPS);
    SmartDashboard.putNumber("Shooter/Top Left RPS", getShooterSpeed());
    SmartDashboard.putNumber("Shooter/Bottom Left RPS", bottomLeftMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/Top Right RPS", topRightMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/Bottom Right RPS", bottomRightMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putBoolean("Shooter/At Speed", atTargetSpeed(2.0)); // Example tolerance
  }
}