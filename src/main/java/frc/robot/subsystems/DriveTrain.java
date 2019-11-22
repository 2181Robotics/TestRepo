/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveTrainDefault;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class DriveTrain extends Subsystem {
  private static final int k_ticks_per_rev = 360;
  private static final double k_wheel_diameter = 6.0 * 2.54 / 100;
  private static final double k_max_velocity = 1;

  private static final int k_left_channel = 5;
  private static final int k_left_channel2 = 3;
  private static final int k_right_channel = 6;
  private static final int k_right_channel2 = 2;

  private static final int k_left_encoder_port_a = 0;
  private static final int k_left_encoder_port_b = 1;
  private static final int k_right_encoder_port_a = 2;
  private static final int k_right_encoder_port_b = 3;

  private static final int k_gyro_port = 0;

  //private static final String k_path_name = "LSO1";
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private Encoder left_encoder;
  private Encoder right_encoder;
  private TalonSRX left_motor;
  private SpeedController left_motor2;
  private SpeedControllerGroup left;
  private TalonSRX right_motor;
  private SpeedController right_motor2;
  private SpeedControllerGroup right;
  private AnalogGyro gyro;
  private EncoderFollower left_follower;
  private EncoderFollower right_follower;
  private Notifier notify;
  private boolean pathing = false;
  private int dir = 1;

  public DriveTrain () {
    super();
    left_motor = new TalonSRX(k_left_channel);
    left_motor.enableBrakeMode(true);
    // left_motor2 = new Talon(k_left_channel2);
    //left = new SpeedControllerGroup(left_motor);//, left_motor2);
    right_motor = new TalonSRX(k_right_channel);
    right_motor.enableBrakeMode(true);
    // right_motor2 = new Talon(k_right_channel2);
    //right = new SpeedControllerGroup(right_motor);//, right_motor2);
    left_motor.setInverted(true);
    left_encoder = new Encoder(k_left_encoder_port_a, k_left_encoder_port_b);
    right_encoder = new Encoder(k_right_encoder_port_a, k_right_encoder_port_b);
    right_encoder.setReverseDirection(true);
    gyro = new AnalogGyro(k_gyro_port);
  }

  @Override
  protected void initDefaultCommand() {
    // TODO Auto-generated method stub
    setDefaultCommand(new DriveTrainDefault());
  }

  public void drive(double speed, double rot) {
    SmartDashboard.putNumber("speedR", right_encoder.getDistance());
    SmartDashboard.putNumber("speedL", left_encoder.getDistance());
    left_motor.set(ControlMode.PercentOutput, (-speed+rot)/2);
    right_motor.set(ControlMode.PercentOutput, (-speed-rot)/2);
    // left.set(.15);
    // left_motor2.set(-.1);
    // right.set(.15);
  }

  public void startPath(Trajectory left_trajectory, Trajectory right_trajectory, boolean forward) {
    if (forward) {
      dir = 1;
      left_encoder.setReverseDirection(false);
      right_encoder.setReverseDirection(true);
    } else {
      dir = -1;
      left_encoder.setReverseDirection(true);
      right_encoder.setReverseDirection(false);
    }
    left_encoder.reset();
    right_encoder.reset();
    left_follower = new EncoderFollower(left_trajectory);
    right_follower = new EncoderFollower(right_trajectory);

    left_follower.configureEncoder(left_encoder.get(), k_ticks_per_rev, k_wheel_diameter);
    // You must tune the PID values on the following line!
    left_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / k_max_velocity, 0);

    right_follower.configureEncoder(right_encoder.get(), k_ticks_per_rev, k_wheel_diameter);
    // You must tune the PID values on the following line!
    right_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / k_max_velocity, 0);
    
    notify = new Notifier(this::followPath);
    notify.startPeriodic(left_trajectory.get(0).dt);
    pathing = true;
  }

  private void followPath() {
    if (left_follower.isFinished() || right_follower.isFinished()) {
      notify.stop();
      pathing = false;
    } else {
      double left_speed = dir*left_follower.calculate(left_encoder.get());
      double right_speed = dir*right_follower.calculate(right_encoder.get());
      double heading = gyro.getAngle();
      double desired_heading = Pathfinder.r2d(left_follower.getHeading());
      double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
      double turn =  0.8 * (-1.0/80.0) * heading_difference;
      turn = 0;
      left_motor.set(ControlMode.PercentOutput, (left_speed + turn)/3.0);
      right_motor.set(ControlMode.PercentOutput, (right_speed - turn)/3.0);
    }
  }

  public boolean pathDone() {
    return !pathing;
  }
}
