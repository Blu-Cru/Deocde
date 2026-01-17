package org.firstinspires.ftc.teamcode.purePursuit.Robot.Commands.DrivetrainCommands;

import android.os.Build;

import androidx.annotation.RequiresApi;

import org.firstinspires.ftc.teamcode.purePursuit.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.purePursuit.Purepursuit.AStar.AStar;
import org.firstinspires.ftc.teamcode.purePursuit.Purepursuit.AStar.Circle;
import org.firstinspires.ftc.teamcode.purePursuit.Purepursuit.CurvePoint;
import org.firstinspires.ftc.teamcode.purePursuit.Robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.purePursuit.Robot.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.purePursuit.Utils.ExtraUtils;

import java.util.ArrayList;

public class AStarpursuit extends Command {


	Robot robot;
	DrivePurePursuit purePursuit;
	AStar a_star;

	@RequiresApi(api = Build.VERSION_CODES.N)
	public AStarpursuit(Robot robot, CurvePoint start, CurvePoint end) {
		this.robot = robot;
		a_star = new AStar(end,start);
		a_star.addCircleObstacle(-24,36,20);
		a_star.addCircleObstacle(-12,36,20);
		a_star.addCircleObstacle(12,36,22);



		ArrayList<CurvePoint> path =  a_star.computeAStar();

		a_star.makePathStable(path);


		purePursuit = new DrivePurePursuit(robot, path);


	}

	@Override
	public void init() {


		purePursuit.init();
	}

	@RequiresApi(api = Build.VERSION_CODES.N)
	@Override
	public void periodic() {
		for (Circle obstacle: a_star.obstacles) {
			ExtraUtils.drawCircle(obstacle,Dashboard.packet);
		}
		purePursuit.periodic();
	}

	@Override
	public boolean completed() {
		return purePursuit.completed();
	}

	@Override
	public void shutdown() {
		purePursuit.shutdown();
	}


}
