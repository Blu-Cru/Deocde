package org.firstinspires.ftc.teamcode.purePursuit.Robot.Commands.DrivetrainCommands;

import org.firstinspires.ftc.teamcode.purePursuit.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.purePursuit.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.purePursuit.Robot.Subsystems.Input;

public class DriveTeleop extends Command {

	Input gamepad1;
	Drivetrain drivetrain;

	public DriveTeleop(Input gamepad1, Drivetrain drivetrain) {
		super(gamepad1, drivetrain);
		this.gamepad1 = gamepad1;
		this.drivetrain = drivetrain;
	}

	@Override
	public void init() {

	}

	@Override
	public void periodic() {
		drivetrain.robotRelative(-gamepad1.getLeft_stick_y(),gamepad1.getRight_stick_x());
	}

	@Override
	public boolean completed() {
		return false;
	}

	@Override
	public void shutdown() {

	}
}
