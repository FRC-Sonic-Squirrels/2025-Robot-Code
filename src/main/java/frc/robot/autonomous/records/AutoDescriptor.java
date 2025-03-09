package frc.robot.autonomous.records;

import frc.robot.autonomous.records.ScoringLocation.ReefSide;
import java.util.ArrayList;
import java.util.List;

public record AutoDescriptor(
    List<ScoringLocation> scoringLocations,
    List<CoralStationLocation> coralStationLocations,
    StartingLocation startingLocation) {
  public enum StartingLocation implements OppositeSide<StartingLocation> {
    S1 {
      @Override
      public StartingLocation getOpposite() {
        return S6;
      }
    },
    S2 {
      @Override
      public StartingLocation getOpposite() {
        return S5;
      }
    },
    S3 {
      @Override
      public StartingLocation getOpposite() {
        return S4;
      }
    },
    S4 {
      @Override
      public StartingLocation getOpposite() {
        return S3;
      }
    },
    S5 {
      @Override
      public StartingLocation getOpposite() {
        return S2;
      }
    },
    S6 {
      @Override
      public StartingLocation getOpposite() {
        return S1;
      }
    }
  }

  public List<ScoringLocation> flippedScoringLocations() {
    List<ScoringLocation> newLocations = new ArrayList<>();

    for (ScoringLocation scoringLocation : scoringLocations) {
      newLocations.add(flipScoringLocation(scoringLocation));
    }

    return newLocations;
  }

  private ScoringLocation flipScoringLocation(ScoringLocation location) {
    ReefSide side = location.side();
    ReefSide newSide;

    switch (side) {
      case CA:
        newSide = ReefSide.CB;
        break;
      case CB:
        newSide = ReefSide.CA;
        break;
      case CC:
        newSide = ReefSide.CL;
        break;
      case CD:
        newSide = ReefSide.CK;
        break;
      case CE:
        newSide = ReefSide.CJ;
        break;
      case CF:
        newSide = ReefSide.CI;
        break;
      case CG:
        newSide = ReefSide.CH;
        break;
      case CH:
        newSide = ReefSide.CG;
        break;
      case CI:
        newSide = ReefSide.CF;
        break;
      case CJ:
        newSide = ReefSide.CE;
        break;
      case CK:
        newSide = ReefSide.CD;
        break;
      case CL:
        newSide = ReefSide.CC;
        break;
      default:
        newSide = null;
        break;
    }
    return new ScoringLocation(newSide, location.level());
  }

  public List<CoralStationLocation> flippedCoralStationLocations() {
    List<CoralStationLocation> newLocations = new ArrayList<>();

    for (CoralStationLocation scoringLocation : coralStationLocations) {
      newLocations.add(flipCoralStationLocation(scoringLocation));
    }

    return newLocations;
  }

  public CoralStationLocation flipCoralStationLocation(CoralStationLocation location) {
    switch (location) {
      case IA:
        return CoralStationLocation.ID;
      case IB:
        return CoralStationLocation.IC;
      case IC:
        return CoralStationLocation.IB;
      case ID:
        return CoralStationLocation.IA;
      default:
        return null;
    }
  }
}
