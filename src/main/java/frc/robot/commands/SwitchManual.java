// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.Storage;

// public class ExpelBalls extends CommandBase {
//   private Storage storage;
//   /** Creates a new ExpelBalls. */
//   public ExpelBalls(Storage storageIn) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     storage = storageIn;
//     addRequirements(storage);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     Storage.currentState = Storage.StorageState.EXPELLING;
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     storage.runMotor(false);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     Storage.currentState = Storage.StorageState.NOT_EXPELLING;
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return true;
//   }
// }
