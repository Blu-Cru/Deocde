package org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.AllTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.LeftTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.MiddleTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.RightTransferDownCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.transferCommands.RightTransferUpCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.BallColor;
import org.firstinspires.ftc.teamcode.blucru.common.util.MotifPattern;

import java.util.Arrays;
import java.util.HashMap;

public class ShooterMotifCoordinator {
        private static HashMap<String, Command> mapper;
        private static BallColor leftColor, middleColor, rightColor;
        private static MotifPattern motif;
        private static int waits = 500;
        static {
                mapper = new HashMap<>();
                mapper.put("RED_PURPLE_PURPLE_GREEN_PPG", new SequentialCommandGroup(
                                new LeftTransferUpCommand(),
                                new MiddleTransferUpCommand(),
                                new WaitCommand(waits),
                                new RightTransferUpCommand()));
                mapper.put("RED_PURPLE_PURPLE_GREEN_PGP", new SequentialCommandGroup(
                                new MiddleTransferUpCommand(),
                                new WaitCommand(waits),
                                new RightTransferUpCommand(),
                                new WaitCommand(waits),
                                new LeftTransferUpCommand()));
                mapper.put("RED_PURPLE_PURPLE_GREEN_GPP", new SequentialCommandGroup(
                                new RightTransferUpCommand(),
                                new WaitCommand(waits),
                                new LeftTransferUpCommand(),
                                new MiddleTransferUpCommand()));
                mapper.put("RED_PURPLE_GREEN_PURPLE_PPG", new SequentialCommandGroup(
                                new LeftTransferUpCommand(),
                                new RightTransferUpCommand(),
                                new WaitCommand(waits),
                                new MiddleTransferUpCommand()));
                mapper.put("RED_PURPLE_GREEN_PURPLE_PGP", new SequentialCommandGroup(
                                new LeftTransferUpCommand(),
                                new WaitCommand(waits),
                                new MiddleTransferUpCommand(),
                                new WaitCommand(waits),
                                new RightTransferUpCommand()));
                mapper.put("RED_PURPLE_GREEN_PURPLE_GPP", new SequentialCommandGroup(
                                new MiddleTransferUpCommand(),
                                new WaitCommand(waits),
                                new LeftTransferUpCommand(),
                                new RightTransferUpCommand()));
                mapper.put("RED_GREEN_PURPLE_PURPLE_PPG", new SequentialCommandGroup(
                                new RightTransferUpCommand(),
                                new MiddleTransferUpCommand(),
                                new WaitCommand(waits),
                                new LeftTransferUpCommand()));
                mapper.put("RED_GREEN_PURPLE_PURPLE_PGP", new SequentialCommandGroup(
                                new MiddleTransferUpCommand(),
                                new WaitCommand(waits),
                                new LeftTransferUpCommand(),
                                new WaitCommand(waits),
                                new RightTransferUpCommand()));
                mapper.put("RED_GREEN_PURPLE_PURPLE_GPP", new SequentialCommandGroup(
                                new LeftTransferUpCommand(),
                                new WaitCommand(waits),
                                new MiddleTransferUpCommand(),
                                new RightTransferUpCommand()));
                mapper.put("BLUE_PURPLE_PURPLE_GREEN_PPG", new SequentialCommandGroup(
                                new MiddleTransferUpCommand(),
                                new LeftTransferUpCommand(),
                                new WaitCommand(waits),
                                new RightTransferUpCommand()

                ));
                mapper.put("BLUE_PURPLE_PURPLE_GREEN_PGP", new SequentialCommandGroup(
                                new MiddleTransferUpCommand(),
                                new WaitCommand(waits),
                                new RightTransferUpCommand(),
                                new WaitCommand(waits),
                                new MiddleTransferUpCommand()));
                mapper.put("BLUE_PURPLE_PURPLE_GREEN_GPP", new SequentialCommandGroup(
                                new RightTransferUpCommand(),
                                new WaitCommand(waits),
                                new LeftTransferUpCommand(),
                                new MiddleTransferUpCommand()));
                mapper.put("BLUE_PURPLE_GREEN_PURPLE_PPG", new SequentialCommandGroup(
                                new LeftTransferUpCommand(),
                                new RightTransferUpCommand(),
                                new WaitCommand(waits),
                                new MiddleTransferUpCommand()));
                mapper.put("BLUE_PURPLE_GREEN_PURPLE_PGP", new SequentialCommandGroup(
                                new LeftTransferUpCommand(),
                                new WaitCommand(waits),
                                new MiddleTransferUpCommand(),
                                new WaitCommand(waits),
                                new RightTransferUpCommand()));
                mapper.put("BLUE_PURPLE_GREEN_PURPLE_GPP", new SequentialCommandGroup(
                                new MiddleTransferUpCommand(),
                                new WaitCommand(waits),
                                new LeftTransferUpCommand(),
                                new RightTransferUpCommand()));
                mapper.put("BLUE_GREEN_PURPLE_PURPLE_PPG", new SequentialCommandGroup(
                                new MiddleTransferUpCommand(),
                                new RightTransferUpCommand(),
                                new WaitCommand(waits),
                                new LeftTransferUpCommand()));
                mapper.put("BLUE_GREEN_PURPLE_PURPLE_PGP", new SequentialCommandGroup(
                                new MiddleTransferUpCommand(),
                                new WaitCommand(waits),
                                new LeftTransferUpCommand(),
                                new WaitCommand(waits),
                                new RightTransferUpCommand()));
                mapper.put("BLUE_GREEN_PURPLE_PURPLE_GPP", new SequentialCommandGroup(
                                new LeftTransferUpCommand(),
                                new WaitCommand(waits),
                                new RightTransferUpCommand(),
                                new MiddleTransferUpCommand()));
        }

        public static Command getOrderToShoot(Alliance alliance) {
                if (motif == null || leftColor == null || middleColor == null || rightColor == null) {
                        return new SequentialCommandGroup(
                                        new AllTransferUpCommand());
                }
                String key = alliance.toString() + "_" + leftColor.toString() + "_" + middleColor.toString() + "_"
                                + rightColor.toString() + "_" + motif.toString();
                if (!mapper.containsKey(key)) {
                        return new SequentialCommandGroup(
                                        new AllTransferUpCommand());
                }
                return mapper.get(key);
        }

        public static void setLeftColor(BallColor color) {
                leftColor = color;
        }

        public static void setMiddleColor(BallColor color) {
                middleColor = color;
        }

        public static void setRightColor(BallColor color) {
                rightColor = color;
        }

        public static void clear() {
                leftColor = BallColor.UNKNOWN;
                middleColor = BallColor.UNKNOWN;
                rightColor = BallColor.UNKNOWN;
        }

        public static BallColor getLeftColor() {
                return leftColor;
        }

        public static BallColor getMiddleColor() {
                return middleColor;
        }

        public static BallColor getRightColor() {
                return rightColor;
        }

        public static void setMotif(MotifPattern motif) {
                ShooterMotifCoordinator.motif = motif;
        }

        public static MotifPattern getMotif() {
                return motif;
        }

}
