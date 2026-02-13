package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class ShooterSubsystem extends SubsystemBase {

  // Left is the LEADER — all PID and velocity commands go here
  private final TalonFX leftShooterMotor;
  // Right is the FOLLOWER — mirrors the leader, physically spins opposite direction
  private final TalonFX rightShooterMotor;

  private final VelocityVoltage velocityRequest;

  private final Timer atSpeedTimer = new Timer();
  private static final double REQUIRED_TIME_AT_SPEED = 0.2;

  private double targetRPS = 0;

  public ShooterSubsystem() {
    leftShooterMotor  = new TalonFX(52);
    rightShooterMotor = new TalonFX(40);

    velocityRequest = new VelocityVoltage(0).withSlot(0);

    leftShooterMotor.getConfigurator().apply(Configs.shootingMotor.shootingLeftConfig);
    rightShooterMotor.getConfigurator().apply(Configs.shootingMotor.shootingRightConfig);

    // Set the right motor to follow the left motor.
    // MotorAlignmentValue.Opposed means the follower will spin in the 
    // physically opposite direction from the leader — correct for a 
    // standard shooter where motors face each other.
    // If your wheels are spinning the wrong way, swap this to .Aligned.
    rightShooterMotor.setControl(
      new Follower(leftShooterMotor.getDeviceID(), MotorAlignmentValue.Opposed)
    );
  }

  public void setVelocity(double rps) {
    targetRPS = rps;
    leftShooterMotor.setControl(velocityRequest.withVelocity(rps));
    // rightShooterMotor follows automatically 
  }

  /** Stop both motors */
  public void stop() {
    targetRPS = 0;
    atSpeedTimer.stop();
    atSpeedTimer.reset();
    leftShooterMotor.stopMotor();
    // Follower will follow the leader into neutral automatically.
    // But re-applying the follower request after stop() is safer
    rightShooterMotor.setControl(
      new Follower(leftShooterMotor.getDeviceID(), MotorAlignmentValue.Opposed)
    );
  }

  public double getLeftSpeed() {
    return leftShooterMotor.getVelocity().getValueAsDouble();
  }

  public double getRightSpeed() {
    return rightShooterMotor.getVelocity().getValueAsDouble();
  }

  public boolean atTargetSpeed(double tolerance) {
    boolean withinTolerance =
        Math.abs(getLeftSpeed()  - targetRPS) < tolerance &&
        Math.abs(getRightSpeed() - targetRPS) < tolerance;

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

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Target RPS", targetRPS);
    SmartDashboard.putNumber("Shooter Left RPS",   getLeftSpeed());
    SmartDashboard.putNumber("Shooter Right RPS",  getRightSpeed());
  }
}