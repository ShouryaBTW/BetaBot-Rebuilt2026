package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class FloorSubsystem extends SubsystemBase {
  private final TalonFX topFloorMotor;
  private final TalonFX bottomFloorMotor;

  private final VelocityVoltage velocityRequest;

  private double targetRPS = 0;

  public FloorSubsystem() {
    bottomFloorMotor = new TalonFX(31);
    topFloorMotor = new TalonFX(32);

    velocityRequest = new VelocityVoltage(0).withSlot(0);

    topFloorMotor.getConfigurator().apply(Configs.floorMotor.floorConfig);
    bottomFloorMotor.getConfigurator().apply(Configs.floorMotor.floorConfig);
    
    bottomFloorMotor.setControl(   
      new Follower(topFloorMotor.getDeviceID(), MotorAlignmentValue.Aligned)
    );
  }

  public void setVelocity(double rps) {
    targetRPS = rps;
    topFloorMotor.setControl(velocityRequest.withVelocity(rps));
  }

  public void stop() {
    targetRPS = 0;
    topFloorMotor.stopMotor();
  }

  public double getFloorSpeed() {
    return topFloorMotor.getVelocity().getValueAsDouble();
  }

  public boolean atTargetSpeed(double tolerance) {
    boolean withinTolerance = (Math.abs(getFloorSpeed() - targetRPS) < tolerance);
    return withinTolerance;
  }

  public double getTargetRPS() {
    return targetRPS;
  }
  
  @Override
  public void periodic() {
  }
}