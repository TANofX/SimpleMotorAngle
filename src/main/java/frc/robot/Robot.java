// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This sample program shows how to control a motor using a joystick. In the operator control part
 * of the program, the joystick is read and the value is written to the motor.
 *
 * <p>Joystick analog values range from -1 to 1 and motor controller inputs also range from -1 to 1
 * making it easy to work together.
 *
 * <p>In addition, the encoder value of an encoder connected to ports 0 and 1 is consistently sent
 * to the Dashboard.
 */
public class Robot extends TimedRobot {
  private static final double MAX_SPEED = 0.25;
  private static final double MIN_SPEED = -0.25;
  private static final double WHEEL_ERROR_TOLERANCE_RADIANS = 0.02;

  private static final double WHEEL_ROTATIONS_PER_MOTOR_ROTATION = (1.0 / (150.0 / 7.0));

  private static final int kMotorPort = 15;
  private static final int kJoystickPort = 0;

  private final SparkFlex m_motor;
  private final XboxController m_joystick;
  private final RelativeEncoder m_encoder;

  private double wheelAngle = 0.0;

  /** Called once at the beginning of the robot program. */
  public Robot() {
    m_motor = new SparkFlex(kMotorPort, MotorType.kBrushless);
    m_joystick = new XboxController(kJoystickPort);
    m_encoder = m_motor.getEncoder();
  }

  /*
   * The RobotPeriodic function is called every control packet no matter the
   * robot mode.
   */
  @Override
  public void robotPeriodic() {
    wheelAngle = Rotation2d.fromRotations(m_encoder.getPosition() * WHEEL_ROTATIONS_PER_MOTOR_ROTATION).getRadians();

    SmartDashboard.putNumber("Encoder", m_encoder.getPosition());
    SmartDashboard.putNumber("Joystick", m_joystick.getLeftY());
    SmartDashboard.putNumber("Motor", m_motor.getAppliedOutput());
    SmartDashboard.putNumber("Stick Angle (Radians)", calculateStickAngle());
    SmartDashboard.putNumber("Wheel Angle (Radians)", wheelAngle);
  }

  /** The teleop periodic function is called every control packet in teleop. */
  @Override
  public void teleopPeriodic() {
    // Get the joystick angle and convert it to a setpoint for the motor
    Rotation2d rotation = Rotation2d.fromDegrees(calculateStickAngle());

    // Calculate angle error between motor setpoint and current wheel angle
    double angleError = MathUtil.angleModulus(rotation.getRadians() - wheelAngle);

    if (Math.abs(angleError) < WHEEL_ERROR_TOLERANCE_RADIANS) {
      m_motor.set(0.0);
    } else {
      double motorSpeed = MathUtil.clamp(angleError, MIN_SPEED, MAX_SPEED);

      m_motor.set(motorSpeed);
    }
  }

  private double calculateStickAngle() {
    double x = m_joystick.getLeftX();
    double y = m_joystick.getLeftY();

    double angle = Math.atan2(y, x);
    angle += Math.PI / 2; // Adjust for the joystick orientation

    return angle;
  }
}
