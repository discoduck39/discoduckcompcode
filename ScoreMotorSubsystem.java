package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ScoreMotorConstants;
import java.util.function.DoubleSupplier;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;




public class ScoreMotorSubsystem extends SubsystemBase {

private final SparkMax ScoreMotor4;

public ScoreMotorSubsystem(){

    // New motor for the coral inatake and scorer//

    ScoreMotor4 = new SparkMax(ScoreMotorConstants.kScoreMotor4CanId, MotorType.kBrushless);

    // sets a small delay before activation//
    ScoreMotor4.setCANTimeout(50);

    // sets config for ScoreMotor//

    SparkMaxConfig scoreMotor4Config = new SparkMaxConfig();
    scoreMotor4Config.voltageCompensation(ScoreMotorConstants.Score_MOTOR4_VOLTAGE_COMP);
    scoreMotor4Config.smartCurrentLimit(ScoreMotorConstants.Score_Motor4_CURRENT_LIMIT);
    ScoreMotor4.configure(scoreMotor4Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

}

public void periodic(){

}

// allows binding of a button in RobotContainer//

public Command runRoller(
      ScoreMotorSubsystem scoreMotorSubsystem, DoubleSupplier forward, DoubleSupplier reverse) {
    return Commands.run(
        () -> ScoreMotor4.set(forward.getAsDouble() - reverse.getAsDouble()), scoreMotorSubsystem);
  }
    
}
