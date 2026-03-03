package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class SetPivotPosition extends Command {
  public final IntakeSubsystem intakeSubsystem;
  public final double targetPosition;

  public final double TOLERANCE_DEGREES = 2.0;

  public SetPivotPosition(IntakeSubsystem intakeSubsystem, double targetPosition) {
    this.intakeSubsystem = intakeSubsystem;
    this.targetPosition = targetPosition;

    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    intakeSubsystem.setPivotAngle(targetPosition);
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.holdPivot();
  }

  @Override
  public boolean isFinished() {
        return intakeSubsystem.pivotAtTarget(targetPosition, TOLERANCE_DEGREES);
  }
}
