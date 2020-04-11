package ca.mcgill.ecse211.playingfield;

/**s
 * Represents a coordinate point on the competition map grid.
 * 
 * @author Younes Boubekeur
 */
public class Point {
  /** The x coordinate. */
  public double x;

  /** The y coordinate. */
  public double y;

  /**
   * Constructs a Point.
   * 
   * @param x the x coordinate
   * @param y the y coordinate
   */
  public Point(double x, double y) {
    this.x = x;
    this.y = y;
  }
  
  @Override
  public boolean equals(Object o) {
    if (!(o instanceof Point)) {
      return false;
    }
    
    Point other = (Point) o;
    return x == other.x && y == other.y;
  }
  
  @Override
  public final int hashCode() {
    return (int) (100 * x + y);
  }

  @Override
  public String toString() {
    return "(" + x + ", " + y + ")";
  }

}
