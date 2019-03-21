/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import java.util.logging.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.TalonSRXGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;



public class Robot extends TimedRobot {
  private Joystick joystick = new Joystick(1);
  private Joystick joystick1 = new Joystick(0);

  private static final int deviceID = 1;
  private CANSparkMax m_motor;
  private DifferentialDrive differentialDrive;
  private CANPIDController m_pidController;
  private CANEncoder m_encoder;
  private double rotations = 0;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  @Override
  public void disabledInit() {
      m_pidController.setReference(0, ControlType.kPosition);
      rotations = 0;
      m_encoder.setPosition(0);
      }

  @Override
  public void robotInit() {
    differentialDrive = new DifferentialDrive(
      new TalonSRXGroup(new WPI_TalonSRX(2), new WPI_TalonSRX(3),new WPI_TalonSRX(4)),
      new TalonSRXGroup(new WPI_TalonSRX(5), new WPI_TalonSRX(6), new WPI_TalonSRX(7)));
    // initialize motor
    m_motor = new CANSparkMax(7, MotorType.kBrushless);

    /**
     * The restoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_motor.restoreFactoryDefaults();

    /**
     * In order to use PID functionality for a controller, a CANPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_pidController = m_motor.getPIDController();

    // Encoder object created to display position values
    m_encoder = m_motor.getEncoder();

    // PID coefficients
    kP = .1; 
    kI = 1e-4;
    kD = 1.5; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
  
  }
  @Override
  public void disabledPeriodic() {
    SmartDashboard.putNumber("SetPointValue", rotations);
    SmartDashboard.putNumber("Encoder" , m_encoder.getPosition());
  }
  @Override
  public void autonomousPeriodic() {
    if(1<turn && turn <= 2){
      double valueR = 1/turn;
    differentialDrive.tankDrive(joystick1.getRawAxis(0) *valueR ,joystick1.getRawAxis(1));
    }
  else if(0 <= turn && turn < 1){
    double valueL = 1/(2 - turn);
    differentialDrive.tankDrive(joystick1.getRawAxis(0), joystick1.getRawAxis(1) * valueL)
    }
  }
  @Override
  public void teleopPeriodic() {
    differentialDrive.arcadeDrive((Math.pow(-joystick1.getRawAxis(1), 3)), Math.pow(-joystick1.getRawAxis(4), 3));
    // read PID coefficients from SmartDashboard
   
    

    // if PID coefficients on SmartDashboard have changed, write new values to controller

    

    /**
     * PIDController objects are commanded to a set point using the 
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four 
     * parameters:
     *  com.revrobotics.ControlType.kDutyCycle
     *  com.revrobotics.ControlType.kPosition
     *  com.revrobotics.ControlType.kVelocity
     *  com.revrobotics.ControlType.kVoltage
     */
    if(joystick.getRawAxis(3) > .15 && rotations < 35){ 
    rotations = rotations + .4;
    m_pidController.setReference(rotations, ControlType.kPosition);
  }
  else if(joystick.getRawAxis(3) < -.15 && rotations > -20){
    rotations = rotations - .4;
    m_pidController.setReference(rotations, ControlType.kPosition);
  }

    
    SmartDashboard.putNumber("SetPoint", rotations);
    SmartDashboard.putNumber("ProcessVariable", m_encoder.getPosition());
  }
}

