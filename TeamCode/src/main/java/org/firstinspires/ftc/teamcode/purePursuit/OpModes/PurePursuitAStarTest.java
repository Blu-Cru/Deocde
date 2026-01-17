package org.firstinspires.ftc.teamcode.purePursuit.OpModes;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.purePursuit.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.purePursuit.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.purePursuit.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.purePursuit.Purepursuit.CurvePoint;
import org.firstinspires.ftc.teamcode.purePursuit.Robot.Commands.DrivetrainCommands.AStarpursuit;


@Autonomous
public class PurePursuitAStarTest extends BaseAuto {
	@RequiresApi(api = Build.VERSION_CODES.N)
	@Override
	public Command setupAuto(CommandScheduler scheduler) {
		return new AStarpursuit(robot,new CurvePoint(0,0), new CurvePoint(10,60))
				.addNext(turn(0))
				.addNext(new AStarpursuit(robot, new CurvePoint(10,60), new CurvePoint(0,0)))
				.addNext(turn(0));
	}
}
