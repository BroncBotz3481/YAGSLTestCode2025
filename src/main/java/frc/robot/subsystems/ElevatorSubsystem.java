package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
//import edu.wpi.first.wpilibj.examples.elevatorexponentialsimulation.Constants;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.Supplier;


public class ElevatorSubsystem extends SubsystemBase {
  // This gearbox represents a gearbox containing 4 Vex 775pro motors.
  private final DCMotor m_elevatorGearbox = DCMotor.getNEO(2);

  private final ExponentialProfile m_profile =
      new ExponentialProfile(
          ExponentialProfile.Constraints.fromCharacteristics(
              Constants.kElevatorMaxV, Constants.kElevatorkV, Constants.kElevatorkA));

  private ExponentialProfile.State m_setpoint = new ExponentialProfile.State(0, 0);

  // Standard classes for controlling our elevator
  private final PIDController m_pidController =
      new PIDController(Constants.kElevatorKp, Constants.kElevatorKi, Constants.kElevatorKd);

  ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(
          Constants.kElevatorkS,
          Constants.kElevatorkG,
          Constants.kElevatorkV,
          Constants.kElevatorkA);
  private final Encoder m_encoder =
      new Encoder(Constants.kEncoderAChannel, Constants.kEncoderBChannel);
  private final PWMSparkMax m_motor = new PWMSparkMax(Constants.kMotorPort);

  // Simulation classes help us simulate what's going on, including gravity.
  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          m_elevatorGearbox,
          Constants.kElevatorGearing,
          Constants.kCarriageMass,
          Constants.kElevatorDrumRadius,
          Constants.kMinElevatorHeightMeters,
          Constants.kMaxElevatorHeightMeters,
          true,
          0,
          0.005,
          0.0);
  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);
  private final PWMSim m_motorSim = new PWMSim(m_motor);

  // Create a Mechanism2d visualization of the elevator
  private final Mechanism2d m_mech2d =
      new Mechanism2d(Units.inchesToMeters(10), Units.inchesToMeters(50));
  private final MechanismRoot2d m_mech2dRoot =
      m_mech2d.getRoot("Elevator Root", Units.inchesToMeters(5), Units.inchesToMeters(0.5));
  private final MechanismLigament2d m_elevatorMech2d =
      m_mech2dRoot.append(
          new MechanismLigament2d("Elevator",5, 90));

  /** Subsystem constructor. */
  public ElevatorSubsystem(){
    SmartDashboard.putData("el",m_mech2d);
    m_elevatorSim.setInput(m_motorSim.getSpeed() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_encoderSim.setDistance(m_elevatorSim.getPositionMeters());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  public Command setElevator(double goal) {
    return run(()-> {
      var goalState = new ExponentialProfile.State(goal, 0);

      var next = m_profile.calculate(0.020, m_setpoint, goalState);

      // With the setpoint value we run PID control like normal
      double pidOutput = m_pidController.calculate(m_encoder.getDistance(), m_setpoint.position);
      double feedforwardOutput =
              m_feedforward.calculateWithVelocities(m_setpoint.velocity, next.velocity);

      m_motor.setVoltage(pidOutput + feedforwardOutput);

      m_setpoint = next;
    });

  }

  /** Stop the control loop and motor output. */
  public double stop() {
    m_motor.set(0.0);
    return 0;
  }

  /** Reset Exponential profile to begin from current position on enable. */
  public void reset() {
    m_setpoint = new ExponentialProfile.State(m_encoder.getDistance(), 0);
  }

  /** Update telemetry, including the mechanism visualization. */
  public void simulationPeriodic() {
    // Update elevator visualization with position
    m_elevatorMech2d.setLength(Constants.kElevatorMinimumLength + m_encoder.getDistance());
  }

  public void close() {
    m_encoder.close();
    m_motor.close();
    m_mech2d.close();
  }
//nnnnn
}