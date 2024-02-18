package frc.robot.lightstrip;

public class Range {
  private int start;
  private int end;

  public Range(int kStart, int kEnd) {
    start = kStart;
    end = kEnd;
  }

  public int getStart() {
    return start;
  }

  public int getEnd() {
    return end;
  }

  public boolean equals(Range range) {
    return range.start == start && range.end == end;
  }
}
