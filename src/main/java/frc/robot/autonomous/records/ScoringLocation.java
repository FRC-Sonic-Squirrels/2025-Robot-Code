package frc.robot.autonomous.records;

import frc.robot.RobotStates.ScoringLevel;

public record ScoringLocation(ReefSide side, ScoringLevel level) {
  public enum ReefSide implements OppositeSide<ReefSide> {
    CA {
      @Override
      public ReefSide getOpposite() {
        return CB;
      }
    },
    CB {
      @Override
      public ReefSide getOpposite() {
        return CA;
      }
    },
    CC {
      @Override
      public ReefSide getOpposite() {
        return CL;
      }
    },
    CD {
      @Override
      public ReefSide getOpposite() {
        return CK;
      }
    },
    CE {
      @Override
      public ReefSide getOpposite() {
        return CJ;
      }
    },
    CF {
      @Override
      public ReefSide getOpposite() {
        return CI;
      }
    },
    CG {
      @Override
      public ReefSide getOpposite() {
        return CH;
      }
    },
    CH {
      @Override
      public ReefSide getOpposite() {
        return CG;
      }
    },
    CI {
      @Override
      public ReefSide getOpposite() {
        return CF;
      }
    },
    CJ {
      @Override
      public ReefSide getOpposite() {
        return CE;
      }
    },
    CK {
      @Override
      public ReefSide getOpposite() {
        return CD;
      }
    },
    CL {
      @Override
      public ReefSide getOpposite() {
        return CC;
      }
    }
  }
}
