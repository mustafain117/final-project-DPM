package ca.mcgill.ecse211.playingfield;

/**
 * Represents a region on the competition map grid, delimited by its lower-left and upper-right
 * corners (inclusive).
 * 
 * @author Younes Boubekeur
 */
public class Region {
  /** The lower left corner of the region. */
  public Point ll;
  
  /** The upper right corner of the region. */
  public Point ur;
  
  /**
   * Constructs a Region.
   * 
   * @param lowerLeft the lower left corner of the region
   * @param upperRight the upper right corner of the region
   */
  public Region(Point lowerLeft, Point upperRight) {
    validateCoordinates(lowerLeft, upperRight);
    ll = lowerLeft;
    ur = upperRight;
  }
  
  /**
   * Returns the width of the region (length in x-direction) in tile-lengths (feet).
   * 
   * @return the width of the region
   */
  public double getWidth() {
    return ur.x - ll.x;
  }
  
  /**
   * Returns the height of the region (length in y-direction) in tile-lengths (feet).
   * 
   * @return the height of the region
   */
  public double getHeight() {
    return ur.y - ll.y;
  }
  
  /**
   * Validates coordinates.
   * 
   * @param lowerLeft the lower left corner of the region
   * @param upperRight the upper right corner of the region
   */
  private void validateCoordinates(Point lowerLeft, Point upperRight) {
    if (lowerLeft.x > upperRight.x || lowerLeft.y > upperRight.y) {
      throw new IllegalArgumentException(
          "Upper right cannot be below or to the left of lower left!");
    }
  }
  
  @Override
  public boolean equals(Object o) {
    if (!(o instanceof Region)) {
      return false;
    }
    
    Region other = (Region) o;
    return ll.equals(other.ll) && ur.equals(other.ur);
  }
  
  @Override
  public final int hashCode() {
    return (int) (10000 * ll.hashCode() + ur.hashCode());
  }

  @Override
  public String toString() {
    return "[" + ll + ", " + ur + "]";
  }
  
}
