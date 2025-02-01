// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final ElevatorSubsystem     elevator           = new ElevatorSubsystem();
  private final ArmSubsystem          arm                = new ArmSubsystem();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  //ParallelCommandGroup setElevArm = new ParallelCommandGroup(elevator.setGoal(10),arm.setGoal(60));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings

    DriverStation.silenceJoystickConnectionWarning(true);
    elevator.setDefaultCommand(elevator.setGoal(4));

    arm.setDefaultCommand(arm.setGoal(0));
    configureBindings();

  }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Side View", Constants.sideRobotView);

//    m_driverController.button(1).whileTrue(arm.setGoal(15));
    m_driverController.button(1).whileTrue(elevator.setGoal(3));

    m_driverController.button(2).whileTrue(elevator.setGoal(6));
    m_driverController.button(2).whileTrue(arm.setGoal(45));

    m_driverController.button(3).whileTrue(elevator.setGoal(9));
    m_driverController.button(3).whileTrue(arm.setGoal(90));

    m_driverController.button(4).whileTrue(arm.setGoal(135));

    m_driverController.button(5).whileTrue(arm.runSysIdRoutine());

    m_driverController.button(6).whileTrue(arm.setGoal(70));
    m_driverController.button(6).whileTrue(elevator.setGoal(4));
//    m_driverController.button(6).whileTrue(setElevArm(10, 70));

    elevator.atHeight(5, 0.1).whileTrue(Commands.print("I AM ALIVE, YAAA HAAAAA"));


  }

  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return Commands.none();

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }*/
  public ParallelCommandGroup setElevArm(double goal, double degree)
  {
    return new ParallelCommandGroup(elevator.setGoal(goal), arm.setGoal(degree));
  }


}
