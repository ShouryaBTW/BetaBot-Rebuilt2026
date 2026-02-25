package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class FeederSubsystem extends SubsystemBase {
  private final TalonFX feederMotor;
  private final VelocityVoltage velocityRequest;

  public FeederSubsystem() {
    feederMotor = new TalonFX(33);
    velocityRequest = new VelocityVoltage(0).withSlot(0);

    feederMotor.getConfigurator().apply(Configs.feederMotor.feederConfig);
  }

  public void setVelocity(double rps) {
    feederMotor.setControl(velocityRequest.withVelocity(rps)); 
  }

  public void stop() {
    feederMotor.stopMotor();
  }

  public double getFeederSpeed() {
    return feederMotor.getVelocity().getValueAsDouble();
  }

  public boolean atTargetSpeed(double targetRPS, double tolerance) {
      return Math.abs(getFeederSpeed() - targetRPS) < tolerance;
  }

  @Override
  public void periodic() {
  }
}