// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Pigeon2Subsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RampProcess extends SequentialCommandGroup {
  /** Creates a new RampProcess. */
  public RampProcess(Drivetrain drivetrain, Pigeon2Subsystem pigeon) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new PathForward(drivetrain, pigeon));
    addCommands(new Tuner(drivetrain));
    addCommands(new WaitCommand(3));
    addCommands(new Correcter(drivetrain, pigeon));
  }
}
