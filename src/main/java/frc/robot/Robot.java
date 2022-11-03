// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
// import edu.wpi.first.wpilibj.motorcontrol.Talon;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
// import com.ctre.phoenix.motorcontrol;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {
  // private DifferentialDrive m_robotDrive;
  private final Joystick m_stick = new Joystick(0);
  WPI_TalonSRX mytalon = new WPI_TalonSRX(1);
  WPI_TalonSRX mytalon2 = new WPI_TalonSRX(2);
  MotorControllerGroup m_left = new MotorControllerGroup(mytalon, mytalon2);

  WPI_TalonSRX mytalon3 = new WPI_TalonSRX(3);
  WPI_TalonSRX mytalon4 = new WPI_TalonSRX(4);
  MotorControllerGroup m_right = new MotorControllerGroup(mytalon3, mytalon4);
  DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);

  WPI_TalonSRX mytalon5 = new WPI_TalonSRX(5);
  XboxController xbox = new XboxController(0);

  @Override
  public void robotInit() {
    mytalon.set(ControlMode.PercentOutput, 0);
    mytalon2.set(ControlMode.PercentOutput, 0);
    mytalon3.set(ControlMode.PercentOutput, 0);
    mytalon4.set(ControlMode.PercentOutput, 0);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.

    // FRONT_LEFT = new PWMTalonSRX(1);
		// REAR_LEFT = new PWMTalonSRX(2);
		// FRONT_RIGHT = new PWMTalonSRX(3);
		// REAR_RIGHT = new PWMTalonSRX(4);

    // MotorControllerGroup Group1 = new MotorControllerGroup(mytalon, mytalon2);
    // MotorControllerGroup Group2 = new MotorControllerGroup(mytalon3, mytalon4);
    
    // DifferentialDrive drive = new DifferentialDrive()

		// m_robotDrive = new DifferentialDrive(Group1, Group2);
  }

  @Override
  public void teleopPeriodic() {
    // System.out.println(m_stick.getRawAxis(5));
    m_drive.arcadeDrive(m_stick.getX(), -m_stick.getY());

    if (xbox.getLeftBumper()) {
      mytalon5.set(ControlMode.PercentOutput, 0.5);
    } else if (xbox.getRightBumper()) {
      mytalon5.set(ControlMode.PercentOutput, -0.4);
    } else {
      mytalon5.set(ControlMode.PercentOutput, 0);
    }
    // mytalon4.set(ControlMode.PercentOutput, m_stick.getY());
    // mytalon2.set(ControlMode.PercentOutput, -m_stick.getRawAxis(5));
    // mytalon.set(ControlMode.PercentOutput, -m_stick.getRawAxis(5));
    // System.out.println("Here");
    // System.out.println(m_stick.getX());
    // System.out.println(m_stick.getY());
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    // double a = 0.5;
    // double b = 0.5;
    // m_robotDrive.arcadeDrive(m_stick.getRawAxis(1), m_stick.getRawAxis(2));
    // Talon.class.
  }
}
