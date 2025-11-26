package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import org.firstinspires.ftc.teamcode.blucru.common.pathing.Path;
import org.firstinspires.ftc.teamcode.blucru.common.pathing.PurePursuitSegment;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

public class PurePusuitTest extends BluLinearOpMode {
    Path currentPath;
    @Override
    public void initialize(){
        robot.clear();
        addSixWheel();
    }

    public void periodic(){
        if (driver1.pressedA()){
            currentPath = new TestPurePursuitPath().build().start();
        }
        if (currentPath != null){
            currentPath.run();
        }
    }

}
