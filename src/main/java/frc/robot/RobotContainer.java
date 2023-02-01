// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.CmdDrive;
import frc.robot.commands.ResetSwerveAngs;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmCal;
import frc.robot.subsystems.Drive.DriveCal;
import frc.robot.subsystems.Drive.DriveTrain;
import frc.robot.subsystems.Inputs.InputCal;
import frc.robot.subsystems.Inputs.Inputs;
import frc.robot.subsystems.Sensors.SensorCal;
import frc.robot.subsystems.Sensors.Sensors;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  public Inputs inputs;
  public Sensors sensors;
  public DriveTrain driveTrain;
  public Arm arm;

  public DriveCal dCal;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    dCal = new DriveCal();
    
    inputs = new Inputs(this, new InputCal());
    sensors = new Sensors(this, new SensorCal());
    driveTrain = new DriveTrain(this, dCal);
    arm = new Arm(this, new ArmCal());

    CommandScheduler cs = CommandScheduler.getInstance();
    cs.setDefaultCommand(driveTrain, new CmdDrive(this).ignoringDisable(true));

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  
  private void configureBindings() {
    inputs.resetSwerveZeros.whileTrue(new ResetSwerveAngs(this).ignoringDisable(true));//down on the left blue jog doo-hickey
    inputs.resetAngle.whileTrue(new InstantCommand(sensors::resetNavXAng).ignoringDisable(true));//up on the left blue jog doo-hickey
    inputs.resetPosition.whileTrue(new InstantCommand(sensors::resetBotPos).ignoringDisable(true));//up on the left blue jog doo-hickey
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    return null;
  }
}
