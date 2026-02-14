// package frc.robot.subsystems;

// import com.ctre.phoenix6.controls.Follower;
// import com.ctre.phoenix6.controls.VelocityVoltage;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.MotorAlignmentValue;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Configs;

// public class ShooterSubsystem extends SubsystemBase {

//   // Left is the LEADER — all PID and velocity commands go here
//   private final TalonFX leftShooterMotor;
//   // Right is the FOLLOWER — mirrors the leader, physically spins opposite direction
//   private final TalonFX rightShooterMotor;

//   private final VelocityVoltage velocityRequest;

//   private final Timer atSpeedTimer = new Timer();
//   private static final double REQUIRED_TIME_AT_SPEED = 0.2;

//   private double targetRPS = 0;

//   public ShooterSubsystem() {
//     leftShooterMotor  = new TalonFX(52);
//     rightShooterMotor = new TalonFX(40);

//     velocityRequest = new VelocityVoltage(0).withSlot(0);

//     leftShooterMotor.getConfigurator().apply(Configs.shooterMotor.shooterConfig);
//     rightShooterMotor.getConfigurator().apply(Configs.shooterMotor.shooterConfig);

//     // Set the right motor to follow the left motor.
//     // MotorAlignmentValue.Opposed means the follower will spin in the 
//     // physically opposite direction from the leader — correct for a 
//     // standard shooter where motors face each other.
//     // If your wheels are spinning the wrong way, swap this to .Aligned.
//     rightShooterMotor.setControl(
//       new Follower(leftShooterMotor.getDeviceID(), MotorAlignmentValue.Opposed)
//     );
//   }

//   public void setVelocity(double rps) {
//     targetRPS = rps;
//     leftShooterMotor.setControl(velocityRequest.withVelocity(rps));
//     // rightShooterMotor follows automatically 
//   }

//   /** Stop both motors */
//   public void stop() {
//     targetRPS = 0;
//     atSpeedTimer.stop();
//     atSpeedTimer.reset();
//     leftShooterMotor.stopMotor();
//     // Follower will follow the leader into neutral automatically.
//     // But re-applying the follower request after stop() is safer
//     rightShooterMotor.setControl(
//       new Follower(leftShooterMotor.getDeviceID(), MotorAlignmentValue.Opposed)
//     );
//   }

//   public double getLeftSpeed() {
//     return leftShooterMotor.getVelocity().getValueAsDouble();
//   }

//   public double getRightSpeed() {
//     return rightShooterMotor.getVelocity().getValueAsDouble();
//   }

//   public boolean atTargetSpeed(double tolerance) {
//     boolean withinTolerance =
//         Math.abs(getLeftSpeed()  - targetRPS) < tolerance &&
//         Math.abs(getRightSpeed() - targetRPS) < tolerance;

//     if (withinTolerance) {
//       if (!atSpeedTimer.isRunning()) {
//         atSpeedTimer.restart();
//       }
//     } else {
//       atSpeedTimer.stop();
//       atSpeedTimer.reset();
//     }

//     return atSpeedTimer.hasElapsed(REQUIRED_TIME_AT_SPEED);
//   }

//   @Override
//   public void periodic() {
//     SmartDashboard.putNumber("Shooter Target RPS", targetRPS);
//     SmartDashboard.putNumber("Shooter Left RPS",   getLeftSpeed());
//     SmartDashboard.putNumber("Shooter Right RPS",  getRightSpeed());
//   }
// }

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

  // Left motor is the leader - all PID and velocity commands go here
  private final TalonFX leftShooterMotor;
  // Right motor is the follower - mirrors the leader with opposite direction
  private final TalonFX rightShooterMotor;

  private final VelocityVoltage velocityRequest;

  private final Timer atSpeedTimer = new Timer();
  private static final double REQUIRED_TIME_AT_SPEED = 0.2;

  private double targetRPS = 0;

  public ShooterSubsystem() {
    leftShooterMotor = new TalonFX(52);
    rightShooterMotor = new TalonFX(40);

    velocityRequest = new VelocityVoltage(0).withSlot(0);

    // Apply the same configuration to both motors
    leftShooterMotor.getConfigurator().apply(Configs.shooterMotor.shooterConfig);
    rightShooterMotor.getConfigurator().apply(Configs.shooterMotor.shooterConfig);

    // Set follower AFTER configurations are applied
    // true = opposed direction (motors face each other)
    rightShooterMotor.setControl(
      new Follower(leftShooterMotor.getDeviceID(), MotorAlignmentValue.Opposed)
    );
  }

  /**
   * Set shooter velocity in rotations per second
   * @param rps Target velocity in RPS
   */
  public void setVelocity(double rps) {
    targetRPS = rps;
    leftShooterMotor.setControl(velocityRequest.withVelocity(rps));
    // Right motor follows automatically
  }

  /** Stop both motors */
  public void stop() {
    targetRPS = 0;
    atSpeedTimer.stop();
    atSpeedTimer.reset();
    leftShooterMotor.stopMotor();
    // Follower automatically stops with leader
  }

  /** Get left (leader) motor velocity in RPS */
  public double getLeftSpeed() {
    return leftShooterMotor.getVelocity().getValueAsDouble();
  }

  /** Get right (follower) motor velocity in RPS (will be opposite sign) */
  public double getRightSpeed() {
    return rightShooterMotor.getVelocity().getValueAsDouble();
  }

  /**
   * Check if shooter is at target speed for required duration
   * @param tolerance Allowed error in RPS
   * @return True if at speed for REQUIRED_TIME_AT_SPEED seconds
   */
  public boolean atTargetSpeed(double tolerance) {
    // Only check leader motor since follower mirrors it
    boolean withinTolerance = Math.abs(getLeftSpeed() - targetRPS) < tolerance;

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
    SmartDashboard.putNumber("Shooter/Left RPS", getLeftSpeed());
    SmartDashboard.putNumber("Shooter/Right RPS", getRightSpeed());
    SmartDashboard.putBoolean("Shooter/At Speed", atTargetSpeed(2.0)); // Example tolerance
  }
}