// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static au.grapplerobotics.interfaces.LaserCanInterface.LASERCAN_STATUS_VALID_MEASUREMENT;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.RobotMath.Elevator;

public class ElevatorSubsystem extends SubsystemBase
{

  public final Trigger atMin = new Trigger(() -> getHeight().lte(ElevatorConstants.kMinElevatorHeight));
  public final Trigger atMax = new Trigger(() -> getHeight().gte(ElevatorConstants.kMaxElevatorHeight));


  // This gearbox represents a gearbox containing 1 Neo
  private final DCMotor m_elevatorGearbox = DCMotor.getNEO(1);

  // Standard classes for controlling our elevator
  ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(
          ElevatorConstants.kElevatorkS,
          ElevatorConstants.kElevatorkG,
          ElevatorConstants.kElevatorkV,
          ElevatorConstants.kElevatorkA);
  private final SparkMax                  m_motor      = new SparkMax(1, MotorType.kBrushless);
  private final SparkClosedLoopController m_controller = m_motor.getClosedLoopController();
  private final RelativeEncoder           m_encoder    = m_motor.getEncoder();
  private final SparkMaxSim               m_motorSim   = new SparkMaxSim(m_motor, m_elevatorGearbox);

  // Sensors
  private final LaserCan         m_elevatorLaserCan     = new LaserCan(0);
  private final LaserCanSim      m_elevatorLaserCanSim  = new LaserCanSim(0);
  private final RegionOfInterest m_laserCanROI          = new RegionOfInterest(0, 0, 16, 16);
  private final TimingBudget     m_laserCanTimingBudget = TimingBudget.TIMING_BUDGET_20MS;
  private final Alert            m_laserCanFailure      = new Alert("LaserCAN failed to configure.",
                                                                    AlertType.kError);

  private final DigitalInput m_limitSwitchLow    = new DigitalInput(1);
  private       DIOSim       m_limitSwitchLowSim = null;

  // Simulation classes help us simulate what's going on, including gravity.
  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          m_elevatorGearbox,
          ElevatorConstants.kElevatorGearing,
          ElevatorConstants.kCarriageMass,
          ElevatorConstants.kElevatorDrumRadius,
          ElevatorConstants.kMinElevatorHeight.in(Meters),
          ElevatorConstants.kMaxElevatorHeight.in(Meters),
          true,
          ElevatorConstants.kStartingHeightSim.in(Meters),
          0.01,
          0.0);


  // SysId Routine and seutp
  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage        m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutDistance       m_distance       = Meters.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutLinearVelocity m_velocity       = MetersPerSecond.mutable(0);
  // SysID Routine
  private final SysIdRoutine      m_sysIdRoutine   =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(Volts.per(Second).of(ArmConstants.kArmRampRate), Volts.of(1), Seconds.of(30)),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motor(s).
              m_motor::setVoltage,
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the shooter motor.
                log.motor("elevator")
                   .voltage(
                       m_appliedVoltage.mut_replace(
                           m_motor.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
                   .linearPosition(m_distance.mut_replace(getHeight()))
                   .linearVelocity(m_velocity.mut_replace(getVelocity()));
              },
              this));

  /**
   * Subsystem constructor.
   */
  public ElevatorSubsystem()
  {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .smartCurrentLimit(40)
        .closedLoopRampRate(0.25)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ElevatorConstants.kElevatorKp, ElevatorConstants.kElevatorKi, ElevatorConstants.kElevatorKd)
        .outputRange(-1, 1)
        .maxMotion
        .maxVelocity(Elevator.convertDistanceToRotations(Meters.of(1)).per(Second).in(RPM))
        .maxAcceleration(Elevator.convertDistanceToRotations(Meters.of(2)).per(Second).per(Second)
                                 .in(RPM.per(Second)));
    m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // Publish Mechanism2d to SmartDashboard
    // To view the Elevator visualization, select Network Tables -> SmartDashboard -> Elevator Sim

    try
    {
      m_elevatorLaserCanSim.setRangingMode(RangingMode.LONG);
    } catch (Exception e)
    {
      m_laserCanFailure.set(true);
    }

    if (RobotBase.isSimulation())
    {
      m_limitSwitchLowSim = new DIOSim(m_limitSwitchLow);
      SmartDashboard.putData("Elevator Low Limit Switch", m_limitSwitchLow);
      seedElevatorMotorPosition();
    }
  }

  /**
   * Advance the simulation.
   */
  public void simulationPeriodic()
  {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_elevatorSim.setInput(m_motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_motorSim.iterate(
        Elevator.convertDistanceToRotations(Meters.of(m_elevatorSim.getVelocityMetersPerSecond())).per(Second).in(RPM),
        RoboRioSim.getVInVoltage(),
        0.020);

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));

    // Update lasercan sim.
    m_elevatorLaserCanSim.setMeasurementFullSim(new Measurement(
        LASERCAN_STATUS_VALID_MEASUREMENT,
        (int) (Math.floor(Meters.of(m_elevatorSim.getPositionMeters()).in(Millimeters)) +
               ElevatorConstants.kLaserCANOffset.in(Millimeters)),
        0,
        true,
        m_laserCanTimingBudget.asMilliseconds(),
        m_laserCanROI
    ));


    // Update elevator visualization with position
    Constants.kElevatorTower.setLength(getHeight().in(Meters));
    Constants.kElevatorCarriage.setPosition(ArmConstants.kArmLength, getHeight().in(Meters));
  }

  /**
   * Seed the elevator motor encoder with the sensed position from the LaserCAN which tells us the height of the
   * elevator.
   */
  public void seedElevatorMotorPosition()
  {
    if (RobotBase.isSimulation())
    {
      // Get values from Simulation
      Measurement measurement = m_elevatorLaserCanSim.getMeasurement();
      // Change distance field
      measurement.distance_mm = (int) (Math.floor(Meters.of(m_elevatorSim.getPositionMeters()).in(Millimeters)) -
                                       ElevatorConstants.kLaserCANOffset.in(Millimeters));
      // Update simulation distance field.
      m_elevatorLaserCanSim.setMeasurementFullSim(measurement);

      m_encoder.setPosition(Elevator.convertDistanceToRotations(Millimeters.of(
                                        m_elevatorLaserCanSim.getMeasurement().distance_mm + ElevatorConstants.kLaserCANOffset.in(Millimeters)))
                                    .in(Rotations));
    } else
    {
      m_encoder.setPosition(Elevator.convertDistanceToRotations(Millimeters.of(
                                        m_elevatorLaserCan.getMeasurement().distance_mm + ElevatorConstants.kLaserCANOffset.in(Millimeters)))
                                    .in(Rotations));
    }
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  public void reachGoal(double goal)
  {
    m_controller.setReference(Elevator.convertDistanceToRotations(Meters.of(goal)).in(Rotations),
                              ControlType.kMAXMotionPositionControl,
                              ClosedLoopSlot.kSlot0,
                              m_feedforward.calculate(getVelocity().in(MetersPerSecond)));
  }

  /**
   * Runs the SysId routine to tune the Arm
   *
   * @return SysId Routine command
   */
  public Command runSysIdRoutine()
  {
    return m_sysIdRoutine.dynamic(Direction.kForward).until(atMax)
                         .andThen(m_sysIdRoutine.dynamic(Direction.kReverse)).until(atMin)
                         .andThen(m_sysIdRoutine.quasistatic(Direction.kForward)).until(atMax)
                         .andThen(m_sysIdRoutine.quasistatic(Direction.kReverse)).until(atMin);
  }


  /**
   * Get Elevator Velocity
   *
   * @return Elevator Velocity
   */
  public LinearVelocity getVelocity()
  {
    return Elevator.convertRotationsToDistance(Rotations.of(m_encoder.getVelocity())).per(Minute);
  }

  /**
   * Get the height of the Elevator
   *
   * @return Height of the elevator
   */
  public Distance getHeight()
  {
    return Elevator.convertRotationsToDistance(Rotations.of(m_encoder.getPosition()));
  }

  /**
   * Get the height in meters.
   *
   * @return Height in meters
   */
  public double getHeightMeters()
  {
    return Elevator.convertRotationsToDistance(Rotations.of(m_encoder.getPosition())).in(Meters);
  }

  /**
   * A trigger for when the height is at an acceptable tolerance.
   *
   * @param height    Height in Meters
   * @param tolerance Tolerance in meters.
   * @return {@link Trigger}
   */
  public Trigger atHeight(double height, double tolerance)
  {
    return new Trigger(() -> MathUtil.isNear(height,
                                             getHeightMeters(),
                                             tolerance));
  }

  /**
   * Set the goal of the elevator
   *
   * @param goal Goal in meters
   * @return {@link edu.wpi.first.wpilibj2.command.Command}
   */
  public Command setGoal(double goal)
  {
    return run(() -> reachGoal(goal));
  }

  /**
   * Stop the control loop and motor output.
   */
  public void stop()
  {
    m_motor.set(0.0);
  }

  /**
   * Update telemetry, including the mechanism visualization.
   */
  public void updateTelemetry()
  {
  }

  @Override
  public void periodic()
  {
  }
}