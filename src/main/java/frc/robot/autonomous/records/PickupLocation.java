package frc.robot.autonomous.records;

public enum PickupLocation implements OppositeSide<PickupLocation> {
  IA(false) {
    @Override
    public PickupLocation getOpposite() {
      return ID;
    }
  },
  IB(false) {
    @Override
    public PickupLocation getOpposite() {
      return IC;
    }
  },
  IC(false) {
    @Override
    public PickupLocation getOpposite() {
      return IB;
    }
  },
  ID(false) {
    @Override
    public PickupLocation getOpposite() {
      return IA;
    }
  },
  G1(true) {
    @Override
    public PickupLocation getOpposite() {
      return G3;
    }
  },
  G2(true) {
    @Override
    public PickupLocation getOpposite() {
      return G2;
    }
  },
  G3(true) {
    @Override
    public PickupLocation getOpposite() {
      return G1;
    }
  };

  public final boolean ground;

  PickupLocation(boolean ground) {
    this.ground = ground;
  }
}
