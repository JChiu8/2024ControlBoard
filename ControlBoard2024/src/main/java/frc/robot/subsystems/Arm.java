package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Arm extends ProfiledPIDSubsystem{
    CANSparkMax motor;
    DutyCycleEncoder encoder;

    public Arm() {
        super(
            new ProfiledPIDController(
                ArmConstants.kP, ArmConstants.kI, ArmConstants.kD, new TrapezoidProfile.Constraints(
                    ArmConstants.kMaxVelo,
                    ArmConstants.kMaxAccel)),
                0
        );
        encoder = new DutyCycleEncoder(0);
        motor = new CANSparkMax(2, MotorType.kBrushed);
        encoder.setDistancePerRotation(360.0);
        setGoal(ArmConstants.kArmOffset);
    }

    public void setArmSpeed(double speed) {
        motor.set(speed);
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        motor.setVoltage(output);
    }

    @Override
    public double getMeasurement() {
        return encoder.getDistance() - ArmConstants.kArmOffset;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Encoder Connection", encoder.isConnected());
        SmartDashboard.putNumber("angle", getMeasurement());
        super.periodic();
    }
}
