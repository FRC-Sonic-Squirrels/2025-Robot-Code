package frc.lib.team2930;

import edu.wpi.first.wpilibj.RobotController;

public class ExecutionTiming implements AutoCloseable {
  private static final String ROOT_TABLE = "ExecutionTiming";

  private static LoggerGroup group = LoggerGroup.build(ROOT_TABLE);

  private final LoggerEntry.Decimal logger;
  private long startTime;

  public ExecutionTiming(String context) {
    logger = group.buildDecimal(context);
  }

  public ExecutionTiming start() {
    startTime = RobotController.getFPGATime();
    return this;
  }

  @Override
  public void close() {
    var endTime = RobotController.getFPGATime();

    logger.info(endTime - startTime);
  }
}
