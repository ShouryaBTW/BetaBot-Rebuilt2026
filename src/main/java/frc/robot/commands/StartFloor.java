// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FloorSubsystem;

public class StartFloor extends Command {
  public final FloorSubsystem floorSubsystem;
  public final double floorTargetRPS;

  public StartFloor(FloorSubsystem floorSubsystem, double floorTargetRPS) {
    this.floorSubsystem = floorSubsystem;
    this.floorTargetRPS = floorTargetRPS;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    floorSubsystem.setVelocity(floorTargetRPS);
  }

  @Override
  public void end(boolean interrupted) {
    floorSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
