package org.firstinspires.ftc.teamcode.purePursuit.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.purePursuit.CommandFramework.BaseTeleop;
import org.firstinspires.ftc.teamcode.purePursuit.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.purePursuit.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.purePursuit.Robot.Commands.DrivetrainCommands.CheesyDrive;
import org.firstinspires.ftc.teamcode.purePursuit.Robot.Commands.DrivetrainCommands.ClosedLoopTeleop;
import org.firstinspires.ftc.teamcode.purePursuit.Robot.Commands.DrivetrainCommands.DriveTeleop;
import org.firstinspires.ftc.teamcode.purePursuit.Robot.Subsystems.Input;
import org.firstinspires.ftc.teamcode.purePursuit.Simulation.TestCommandsSubsystems.PrintCommand1;


@TeleOp
public class TestTeleop extends BaseTeleop {

	@Override
	public Command setupTeleop(CommandScheduler scheduler) {
		Command printCommand = new PrintCommand1(robot.print, "Hello, World!");
		robot.gamepad1.whenCrossPressed(printCommand);

		//return new ClosedLoopTeleop(robot.drivetrain,robot.odometry,robot.gamepad1);
		return new CheesyDrive(robot.gamepad1, robot.drivetrain);
	}
}
