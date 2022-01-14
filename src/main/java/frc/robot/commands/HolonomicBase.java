// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.shuffleboard.api.util.Time;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class HolonomicBase extends CommandBase {
  Trajectory m_trajectory;
  Drivetrain m_drivetrain;
  
  double timeThroughTrajectory;
  
  

  
  
  

  


  HolonomicDriveController controller = new HolonomicDriveController(
    new PIDController(1, 0, 0), new PIDController(1, 0, 0), //pids for the x and y velocities
    new ProfiledPIDController(1, 0, 0, //pid for angular velocity
    new TrapezoidProfile.Constraints(6.28, 3.14))); //numbers pulled from the example. Probably need to be changed.

  /** Creates a new HolonomicBase. */
  public HolonomicBase(Drivetrain drivetrain, Trajectory trajectory) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_trajectory = trajectory;
    
    m_drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    
  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    timeThroughTrajectory = timeThroughTrajectory + 0.001; //THIS MIGHT BE WRONG CHECK THIS FIRST IF THINGS GO TO HELL

    Trajectory.State goal = m_trajectory.sample(timeThroughTrajectory); //asks the trajectory where the robot should be at the current time??
    
    //not sure if the desired trajectory state means the desired state at the current moment or if its some kind of waypoint

    ChassisSpeeds adjustedSpeeds = controller.calculate(
    m_drivetrain.getOdometryPose(), goal, Rotation2d.fromDegrees(0)); //figures out what to tell the drivetrain to do based on where it is and needs 2 be

    m_drivetrain.setSpeed(adjustedSpeeds); //tells the drivetrain what to do

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
