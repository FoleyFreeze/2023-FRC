// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Auton.AutonBuilder;
import frc.robot.commands.Auton.AutonPos;
import frc.robot.commands.Auton.AdvancedMovement.MultiDimensionalMotionProfile;
import frc.robot.subsystems.Drive.DriveTrain;
import frc.robot.subsystems.Sensors.Odometry;
import frc.robot.util.Vector;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer r;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    r = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  String prevValue = "";

  @Override
  public void disabledPeriodic() {

    
    /*
    int useSpecialCommand = r.specialAutonChooser.getSelected();

    int startPos = r.startPosChooser.getSelected();
    boolean secondPiece = r.secondPieceChooser.getSelected();
    int action = r.actionChooser.getSelected();
    int path = r.pathChooser.getSelected();
    int piece = r.pieceChooser.getSelected();

    //casts everything to a string
    String value = "" + startPos + secondPiece + action + path + piece;

    if(useSpecialCommand > 0){
      //TODO: make special commands
    }else if(!value.equals(prevValue)){
      r.autonCommand = AutonBuilder.buildAuton(r, startPos, 
                                                  secondPiece, 
                                                  action, 
                                                  path, 
                                                  piece);
    }

*/
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = r.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    /*Vector startPoint = Vector.fromXY(0, 0);
    AutonPos[] waypoints = {new AutonPos(54, 0, 0), new AutonPos(54, 108, 0)};

    AutonPos[] vecs = MultiDimensionalMotionProfile.formulateArcs(16, startPoint, waypoints);

    System.out.println("First Point: " + vecs[0].xy.toStringXY());
    System.out.println("Second Point: " + vecs[1].xy.toStringXY());
    System.out.println("Third Point: " + vecs[2].xy.toStringXY());

    System.out.println("First Length: " + vecs[0].value);
    System.out.println("Second Length: " + vecs[1].value);
    System.out.println("Third Length: " + vecs[2].value);*/
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
