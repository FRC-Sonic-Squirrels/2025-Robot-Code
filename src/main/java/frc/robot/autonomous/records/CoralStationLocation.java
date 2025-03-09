package frc.robot.autonomous.records;

public enum CoralStationLocation implements OppositeSide<CoralStationLocation> {
  IA {
    @Override
    public CoralStationLocation getOpposite() {
      return ID;
    }
  },
  IB {
    @Override
    public CoralStationLocation getOpposite() {
      return IC;
    }
  },
  IC {
    @Override
    public CoralStationLocation getOpposite() {
      return IB;
    }
  },
  ID {
    @Override
    public CoralStationLocation getOpposite() {
      return IA;
    }
  };
}
