package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX rollerMotor;
  private final TalonFX pivotMotor;
  private final VelocityVoltage velocityRequest;
  private final MotionMagicVoltage pivotRequest;

  public IntakeSubsystem() {
    rollerMotor = new TalonFX(30);
    pivotMotor = new TalonFX(50);

    velocityRequest = new VelocityVoltage(0).withSlot(0);
    pivotRequest = new MotionMagicVoltage(0).withSlot(0);

    rollerMotor.getConfigurator().apply(Configs.intakeMotor.rollerConfig);
    pivotMotor.getConfigurator().apply(Configs.intakeMotor.pivotConfig);

    pivotMotor.setPosition(0);
  }

  // ─── Pivot ────────────────────────────────────────────────────────────────

  public void setPivotAngle(double degrees) {
    double rotations = degrees / 360.0 * Configs.intakeMotor.PIVOT_GEAR_RATIO;
    pivotMotor.setControl(pivotRequest.withPosition(rotations));
  }

  public void stopPivot() {
    pivotMotor.stopMotor();
  }

  public void holdPivot() {
    pivotMotor.setControl(pivotRequest.withPosition(pivotMotor.getPosition().getValueAsDouble()));
  }

  public double getPivotAngle() {
    return (pivotMotor.getPosition().getValueAsDouble() / Configs.intakeMotor.PIVOT_GEAR_RATIO) * 360.0;
  }

  public boolean pivotAtTarget(double targetDegrees, double toleranceDegrees) {
    return Math.abs(getPivotAngle() - targetDegrees) < toleranceDegrees;
  }

  // ─── Roller ───────────────────────────────────────────────────────────────

  public void setRollerVelocity(double rps) {
    rollerMotor.setControl(velocityRequest.withVelocity(rps));
  }

  public void stopRoller() {
    rollerMotor.stopMotor();
  }

  public double getRollerSpeed() {
    return rollerMotor.getVelocity().getValueAsDouble();
  }

  public boolean rollerAtTarget(double targetRPS, double tolerance) {
    return Math.abs(getRollerSpeed() - targetRPS) < tolerance;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pivot Position", getPivotAngle());
  }
}