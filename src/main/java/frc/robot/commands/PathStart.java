/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.IOException;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;

public class PathStart extends Command {
  private boolean forward;
  Trajectory left_trajectory;
  Trajectory right_trajectory;

  public PathStart(String path, boolean forward) {
    this.forward = forward;
    
    try {
      if (forward) {
        left_trajectory = PathfinderFRC.getTrajectory(path + ".left");
        right_trajectory = PathfinderFRC.getTrajectory(path + ".right");
      } else {
        left_trajectory = PathfinderFRC.getTrajectory(path + ".right");
        right_trajectory = PathfinderFRC.getTrajectory(path + ".left");
      }
    } catch (IOException e) {
      System.out.println("No path named "+path);
    }
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // Robot.driveTrain.startPath(path, forward);
    if (left_trajectory != null && right_trajectory != null) {
      Robot.driveTrain.startPath(left_trajectory, right_trajectory, forward);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.driveTrain.pathDone();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
