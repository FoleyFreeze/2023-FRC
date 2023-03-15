// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.Auton.AutonBuilder;
import frc.robot.commands.Auton.AutonPos;
import frc.robot.commands.Auton.AdvancedMovement.AngleMotionProfile;
import frc.robot.commands.Auton.AdvancedMovement.DriveMotionProfile;
import frc.robot.commands.Auton.AutonToolbox.AutoBalance;
import frc.robot.commands.Auton.AutonToolbox.AutonCommand;
import frc.robot.commands.Auton.AutonToolbox.SimpleScore;
import frc.robot.commands.Auton.BasicMovement.DriveForTime;
import frc.robot.subsystems.Drive.DriveCal;
import frc.robot.subsystems.Drive.DriveTrain;
import frc.robot.subsystems.Drive.DriveCal.WheelCal;
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
    
    //SmartDashboard.putData("40%",new InstantCommand(()->s.set(0.25)).andThen(new WaitCommand(1)).andThen(()->s.setDisabled()));
    //SmartDashboard.putData("50%",new InstantCommand(()->s.set(0.5)).andThen(new WaitCommand(1)).andThen(()->s.setDisabled()));
    //SmartDashboard.putData("60%",new InstantCommand(()->s.set(0.65)).andThen(new WaitCommand(1)).andThen(()->s.setDisabled()));
  }

  Servo s = new Servo(0);

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

    r.driveTrain.resetWheelReads();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  String prevValue = "";

  @Override
  public void disabledPeriodic() {

    int useSpecialCommand = r.specialAutonChooser.getSelected();

    int autonChooser = r.autonChooser.getSelected();
    int startPos = r.autonStartPosChooser.getSelected();

    /*int simpleStartPos = r.simpleStartPosChooser.getSelected();
    boolean simpleBalance = r.simpleBalanceChooser.getSelected();*/
    Alliance team = DriverStation.getAlliance();

    /*int startPos = r.startPosChooser.getSelected();
    boolean secondPiece = r.secondPieceChooser.getSelected();
    int action = r.actionChooser.getSelected();
    int path = r.pathChooser.getSelected();
    int piece = r.pieceChooser.getSelected();*/

    //casts everything to a string
    String value = "" + autonChooser + startPos + team + useSpecialCommand;// + simpleStartPos + simpleBalance +*/ team + startPos + secondPiece + action + path + piece;

    if(!value.equals(prevValue)){
      if(useSpecialCommand > 0){
        //r.autonCommand = SimpleScore.SimpleHiScore(r, simpleStartPos, simpleBalance, team);
        
      } else {
        r.autonCommand = AutonCommand.autonCommand(r, team, autonChooser, startPos);
        /*r.autonCommand = AutonBuilder.buildAuton(r, team,
                                                    startPos, 
                                                    secondPiece, 
                                                    action, 
                                                    path, 
                                                    piece);*/
      }
    }

    prevValue = value;
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
    r.driveTrain.setParkMode(false);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
  }

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

    //DriveMotionProfile mp = new DriveMotionProfile(r, Vector.fromXY(60,0));
    AngleMotionProfile mp = new AngleMotionProfile(r, Math.PI);
    mp.initialize();
    for(double t=0;t<3;t+=0.02){
      double[] avp = mp.getAVP(t);
      System.out.format("%.2f,%.2f,%.2f,%.2f\n",t,avp[2],avp[1],avp[0]);
    }

    /*Vector startPoint = Vector.fromXY(0, 0);
    AutonPos[] waypoints = {new AutonPos(54, 0, 0), new AutonPos(54, 108, 0)};

    AutonPos[] vecs = MultiDimensionalMotionProfile.formulateArcs(16, startPoint, waypoints);

    System.out.println("First Point: " + vecs[0].xy.toStringXY());
    System.out.println("Second Point: " + vecs[1].xy.toStringXY());
    System.out.println("Third Point: " + vecs[2].xy.toStringXY());

    System.out.println("First Length: " + vecs[0].value);
    System.out.println("Second Length: " + vecs[1].value);
    System.out.println("Third Length: " + vecs[2].value);*/

    /*
    Vector[] wheelLocations = {r.driveTrain.wheels[0].cal.wheelLocation,r.driveTrain.wheels[1].cal.wheelLocation,r.driveTrain.wheels[2].cal.wheelLocation,r.driveTrain.wheels[3].cal.wheelLocation};
    Vector xy = Vector.fromXY(0.0, 0.1);
    double zPwr = 0.0;

    Vector v0 = Vector.fromXY(0.2,0.2);
    Vector v1 = (new Vector(0.4,wheelLocations[0].theta+Math.PI/2)).add(v0);
    Vector v2 = (new Vector(0.4,wheelLocations[1].theta+Math.PI/2)).add(v0);
    Vector v3 = (new Vector(0.4,wheelLocations[2].theta+Math.PI/2)).add(v0);
    Vector v4 = (new Vector(0.4,wheelLocations[3].theta+Math.PI/2)).add(v0);
    Vector[] driveVecs = {v1,v2,v3,v4};

    double[] vals = Odometry.formulateBestValuesMatrix(driveVecs, wheelLocations);
    Vector strafe = new Vector(vals[0], vals[1]);
    double angle = vals[2];
    double error = vals[3];
    */
    
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
