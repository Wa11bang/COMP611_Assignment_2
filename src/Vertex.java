import java.awt.geom.Point2D;

/**
 * An object which encapsulates a Point2D element.
 *
 * @author Waldo Theron 18033655
 */
public class Vertex {
    private final Point2D point; // point that is encapsulated

    /**
     * Constructor for a Vertex object, initialises the Point2D object with given x, y parameters.
     * @param x Point2D coordinate
     * @param y Point2D coordinate
     */
    public Vertex(double x, double y) {
        this.point = new Point2D.Double(x, y);
    }

    /**
     * Returns the encapsulated Point2D's (x) coordinate
     * @return x coordinate
     */
    public double getX() {
        return this.point.getX();
    }

    /**
     * Returns the encapsulated Point2D's (y) coordinate
     * @return y coordinate
     */
    public double getY() {
        return this.point.getY();
    }

    /**
     * This method passes a given Vertex to a calculation function which calculates the distance between two given Vertices.
     * @param vertex to calculate the distance from
     * @return distance between this and another Vertex
     */
    public double distance(Vertex vertex)
    {
        return Vertex.distance(this, vertex);
    }

    /**
     * Method which calculates the distance between two given Vertices on a 2-dimensional plane using PX and PY formulae.
     * @param vertex1 for calculation
     * @param vertex2 for calculation
     * @return distance between two Vertices
     */
    public static double distance(Vertex vertex1, Vertex vertex2)
    {
        double px = vertex1.getX() - vertex2.getX();
        double py = vertex1.getY() - vertex2.getY();
        return Math.sqrt(px * px + py * py);
    }

    /**
     * Customised String representation of a Vertex, simply X and Y concatenated.
     * @return Vertex String representation.
     */
    @Override
    public String toString()
    {
        return (getX() + " " + getY());
    }
}
