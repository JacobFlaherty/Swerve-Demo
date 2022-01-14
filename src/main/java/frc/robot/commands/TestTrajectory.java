// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class TestTrajectory extends HolonomicBase {
  /** Creates a new TestTrajectory. */
  
  
  public TestTrajectory(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(drivetrain, TrajectoryGenerator.generateTrajectory(
      drivetrain.getOdometryPose(),
      List.of(
        new Translation2d(3, 0),
        new Translation2d(3, 3),
        new Translation2d(0, 3)
      ),

      
      new Pose2d(0, 0, Rotation2d.fromDegrees(0))
      , Constants.config)
    );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
