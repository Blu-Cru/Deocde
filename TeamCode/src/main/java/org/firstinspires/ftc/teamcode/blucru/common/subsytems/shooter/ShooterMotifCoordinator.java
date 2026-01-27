package org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.LeftTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.MiddleTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.RightTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.BallColor;

import java.util.Arrays;
import java.util.HashMap;

public class ShooterMotifCoordinator {
    private static HashMap<String, Command> mapper;
    private static BallColor leftColor, middleColor, rightColor;
    static{
        mapper = new HashMap<>();
        mapper.put("RED_PURPLE_PURPLE_GREEN_[p, p, g]", new SequentialCommandGroup(
                new LeftTransferUpCommand(),
                new MiddleTransferUpCommand(),
                new WaitCommand(200),
                new RightTransferUpCommand()
        ));
        mapper.put("RED_PURPLE_PURPLE_GREEN_[p, g, p]", new SequentialCommandGroup(
                new LeftTransferUpCommand(),
                new RightTransferUpCommand(),
                new WaitCommand(200),
                new MiddleTransferUpCommand()
        ));
        //TODO FINISH THE INPUTS
    }

    public static Command getOrderToShoot(Alliance alliance, String[] pattern){
        String key = alliance.toString() + "_" + leftColor.toString() + "_" + middleColor.toString() + "_" + rightColor.toString() + "_" + Arrays.toString(pattern);
        if (!mapper.containsKey(key)){
            return new SequentialCommandGroup(
                    new AllTransferUpCommand()
            );
        }
        return mapper.get(key);
    }

    public static void setLeftColor(BallColor color){
        leftColor = color;
    }

    public static void setMiddleColor(BallColor color){
        middleColor = color;
    }

    public static void setRightColor(BallColor color){
        rightColor = color;
    }

}
