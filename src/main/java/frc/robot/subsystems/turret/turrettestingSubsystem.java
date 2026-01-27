package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class turrettestingSubsystem extends SubsystemBase {

  private static final turrettestingSubsystem INSTANCE = new turrettestingSubsystem();

  public static turrettestingSubsystem getInstance() {
    return INSTANCE;
  }

  private final NetworkTable limelightTable;

  private final TalonFX turretMotor = new TalonFX(3);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  private static final double MAX_ANGLE = 720;
  private static final double MIN_ANGLE = -720;
  private static final double MAX_VELOCITY_IN_DEG_PER_SEC = 60;
  private static final double MAX_ACCELERATION_IN_DEG_PER_SEC = 120;
  private static final double UNWIND_THRESHOLD = 500;
  private static final Rotation2d UNWIND_TARGET = Rotation2d.fromDegrees(0.0);
  private static final double GEAR_RATIO = 60.8; // 1:1

  private final double kP = 2.0; // 2
  private final double kI = 0.1;
  private final double kD = 0.1; // 0.1
  private final double kV = 0.12; // 0.12
  private final double kA = 0.02; // 0.02

  private final TrapezoidProfile profile;
  private TrapezoidProfile.State lastSetpoint;

  private Rotation2d targetAngle;
  private boolean isUnwinding;

  private boolean wrappingAround;

  private double lastTimeAtGoal = 0;

  public turrettestingSubsystem() {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight-three");
    this.profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                Math.toRadians(MAX_VELOCITY_IN_DEG_PER_SEC),
                Math.toRadians(MAX_ACCELERATION_IN_DEG_PER_SEC)));

    this.lastSetpoint = new TrapezoidProfile.State(0.0, 0.0);
    this.targetAngle = new Rotation2d();
    this.isUnwinding = false;

    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    turretMotor.getConfigurator().apply(config);
    turretMotor.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double currentPositionDeg = getAbsolutePositionDeg();
    double currentVelocityDegPerSec = getVelocityDegPerSec();

    double voltage = 0.0;

    if (Math.abs(currentPositionDeg) > UNWIND_THRESHOLD) {
      isUnwinding = true;
      targetAngle = UNWIND_TARGET;
      //      System.out.println("unwinding");
    } else if (isUnwinding && atGoal(currentPositionDeg)) {
      // Finished unwinding
      //      System.out.println(
      //
      // "DONEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE");
      isUnwinding = false;
    } else if (wrappingAround && atGoal(currentPositionDeg)) {
      wrappingAround = false;
    }
    if (!isUnwinding && !wrappingAround) {
      setTarget();
      //      System.out.println("targeting");
    }
    if (isUnwinding) {
      System.out.println("is unwinding");
      targetAngle = UNWIND_TARGET;
    }

    //    System.out.println(targetAngle + "" + atGoal(currentPositionDeg));
    voltage = calculate(currentPositionDeg, currentVelocityDegPerSec, 0.02);
    turretMotor.setControl(voltageRequest.withOutput(voltage));
  }

  public void setTarget() {
    double tx = limelightTable.getEntry("tx").getDouble(0.0);
    double tv = limelightTable.getEntry("tv").getDouble(0.0);

    if (tv < 1) {
      //      System.out.print("no target");

      //      turretMotor.setControl(voltageRequest.withOutput(0.0));
      //      targetAngle = Rotation2d.fromDegrees(getAbsolutePositionDeg());
      targetAngle = targetAngle; // hold position
      return;
    }

    //    // see if we need to unwind
    //    if (Math.abs(turretMotor.getPosition().getValueAsDouble()) > UNWIND_THRESHOLD) {
    //      // need to unwind
    //      isUnwinding = true;
    //      targetAngle = UNWIND_TARGET;
    //    } else {
    // it ok no need to unwind
    //    isUnwinding = false;
    //    System.out.print("has target");
    targetAngle =
        findBestTarget(
            Rotation2d.fromDegrees(tx + getAbsolutePositionDeg()),
            getAbsolutePositionDeg()); // target from vision
    //    }
  }

  private Rotation2d findBestTarget(Rotation2d fieldRelativeTarget, double currentAbsoluteDeg) {
    double wrappedTargetDeg = fieldRelativeTarget.getDegrees();

    double[] canididates = {
      wrappedTargetDeg,
      wrappedTargetDeg + 360,
      wrappedTargetDeg - 360,
      wrappedTargetDeg + 720,
      wrappedTargetDeg - 720
    };

    double bestTarget = currentAbsoluteDeg;
    double smallestError = Double.POSITIVE_INFINITY;

    for (double candidate : canididates) {
      if (candidate < MIN_ANGLE || candidate > MAX_ANGLE) {
        continue;
      }

      double error = Math.abs(candidate - currentAbsoluteDeg);

      if (error < smallestError) {
        smallestError = error;
        bestTarget = candidate;
      }
    }
    System.out.println(smallestError);
    if (smallestError > 300) {
      lastTimeAtGoal = Timer.getFPGATimestamp();
      wrappingAround = true;
    }
    return Rotation2d.fromDegrees(bestTarget);
  }

  public double calculate(
      double currentAngleDeg, double currentVelocityDegPerSec, double deltaTimeSec) {
    currentAngleDeg = MathUtil.clamp(currentAngleDeg, MIN_ANGLE, MAX_ANGLE); // safe it or smth
    targetAngle =
        Rotation2d.fromDegrees(MathUtil.clamp(targetAngle.getDegrees(), MIN_ANGLE, MAX_ANGLE));
    TrapezoidProfile.State goal = new TrapezoidProfile.State(targetAngle.getRadians(), 0.0);

    TrapezoidProfile.State setpoint = profile.calculate(deltaTimeSec, lastSetpoint, goal);

    double velocityError = setpoint.velocity - Math.toRadians(currentVelocityDegPerSec);
    double positionError = setpoint.position - Math.toRadians(currentAngleDeg);

    double accelleration = (setpoint.velocity - lastSetpoint.velocity) / deltaTimeSec;
    System.out.println(
        "position error: "
            + Math.toDegrees(positionError)
            + " velocity error: "
            + Math.toDegrees(velocityError)
            + " setpoint vel: "
            + Math.toDegrees(setpoint.velocity)
            + " acceleration: "
            + Math.toDegrees(accelleration));
    double output =
        kP * positionError + kD * velocityError + kV * setpoint.velocity + kA * accelleration;
    output = MathUtil.clamp(output, -12.0, 12.0);

    lastSetpoint = setpoint;

    return output;

    //    targetAngle =
    //        Rotation2d.fromDegrees(MathUtil.clamp(targetAngle.getDegrees(), MIN_ANGLE,
    // MAX_ANGLE));
    //
    //    // Simple P control (no motion profile for now)
    //    double output = 0;
    //    double positionError = targetAngle.getDegrees() - currentAngleDeg;
    //    if (wrappingAround) {
    //      double timeNotAtGoal = Timer.getFPGATimestamp() - lastTimeAtGoal;
    //      output = kP * positionError + timeNotAtGoal * kI;
    //      System.out.println(timeNotAtGoal * kI + "intergral");
    //    } else {
    //      output = kP * positionError;
    //    }
    //    output = MathUtil.clamp(output, -12.0, 12.0);
    //
    //    //    System.out.println("Error: " + positionError + " Output: " + output);
    //
    //    return output;
  }

  public double getAbsolutePositionDeg() {
    double motorRotations = turretMotor.getPosition().getValueAsDouble();
    return (motorRotations / GEAR_RATIO) * 360.0;
  }

  public double getVelocityDegPerSec() {
    double motorRPS = turretMotor.getVelocity().getValueAsDouble();
    return (motorRPS / GEAR_RATIO) * 360.0;
  }

  public boolean atGoal(double currentPositionDeg) {
    double positionError = Math.abs(targetAngle.getDegrees() - currentPositionDeg);
    double velocityError = Math.abs(getVelocityDegPerSec());
    //    System.out.println(positionError + "   egrnkjnkjrekjgnkj   " + velocityError);
    if (positionError < 3.0 && velocityError < 3.0) {
      lastTimeAtGoal = Timer.getFPGATimestamp();
      wrappingAround = false;
      return true;
    }
    //    return positionError < 25.0 && velocityError < 3.0;
    return false;
  }
}
