/**
 * An object which encapsulates two Vertices which form a bond / line within a polygon.
 *
 * @author Waldo Theron 18033655
 */
public class Edge {
    private final Vertex v1; // Vertex Uno reference
    private final Vertex v2; // Vertex Dos reference

    /**
     * Constructor for an Edge object. Takes in two Vertices which form an edge in a Path or Polygon.
     * @param v1 Vertex Uno
     * @param v2 Vertex Dos
     */
    public Edge(Vertex v1, Vertex v2) {
        this.v1 = v1;
        this.v2 = v2;
    }

    /**
     * Gets the start-of-edge Vertex
     * @return Vertex Uno
     */
    public Vertex getV1() {
        return v1;
    }

    /**
     * Gets the end-of-edge Vertex
     * @return Vertex Dos
     */
    public Vertex getV2() {
        return v2;
    }

    /**
     * Returns a customised String representation of an Edge Object using the encapsulated Vertices.
     * @return Custom String representation of an Edge
     */
    @Override
    public String toString()
    {
        return ("v(" + getV1() + ") --- v(" + getV2() + ")");
    }

}
