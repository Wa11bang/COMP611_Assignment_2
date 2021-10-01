import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

/**
 * A Polygon object encapsulates a Path2D object of exterior edges and a List of Vertices. A Polygon generates a random set
 * of Vertices based on the Valtr Algorithm to always form convex polygons. Three approaches to solve the Tessellation problem
 * are included: Brute Force which works for upto 15 sides effectively, Approximate Greedy which works for upto 100000 sides,
 * and an Exact (Dynamic Programming) approach with works for upto 1000 sides effectively.
 *
 * @author Waldo Theron 18033655
 */
public class Polygon {
    private final int numOfPoints;
    private final Path2D polygonPath;
    private final List<Vertex> vertices;
    private Set<Edge> edges;

    /**
     * Constructor for a Polygon Object which requires an integer used for calculate the number of vertices.
     * @param numOfPoints used to generate a polygon of N vertices.
     */
    public Polygon(int numOfPoints)
    {
        this.numOfPoints = numOfPoints;
        this.edges = new LinkedHashSet<>();
        this.vertices = new LinkedList<>();

        /*
            Generates a list of Point2D.Double objects which form a convex polygon of N points. Uses the ValtrAlgorithm
            conceptualised by Pavel Valtr and implemented by Sander Verdonschot.
         */
        List<Point2D.Double> generatedPoints = ValtrAlgorithm.generateRandomConvexPolygon(numOfPoints);

        for(int i = 0; i < numOfPoints; ++i)
        {
            Point2D.Double point = generatedPoints.get(i);
            vertices.add(new Vertex(((point.getX() * 550) + 100), ((point.getY() * 550) + 50)));
        }

        // build path for polygon using generated coordinates
        polygonPath = new Path2D.Double();
        polygonPath.moveTo(vertices.get(0).getX(), vertices.get(0).getY());
        for(int i = 1; i < numOfPoints; ++i) {
            polygonPath.lineTo(vertices.get(i).getX(), vertices.get(i).getY());
        }
        polygonPath.closePath();
    }

    /**
     * Gets the Path2D Object of the Polygon based on the vertices.
     * @return path of the Polygon
     */
    public Path2D getPolygonPath() {
        return polygonPath;
    }

    /**
     * Gets the list of Vertex objects which make up the Polygon.
     * @return list of vertices
     */
    public List<Vertex> getVertices() {
        return vertices;
    }

    /**
     * Gets the set of Edge objects which define interior edges calculated from either
     * brute force, greedy, or exact approaches. This set is empty if an approach method
     * has not been called on this object.
     * @return set of Edges
     */
    public Set<Edge> getEdges() {
        return edges;
    }

    /**
     * Brute force call method which initialises the internal brute force helper method.
     * @return calculated shortest sum of interior edges to form a correct tessellation within the Polygon path.
     */
    public double bruteForceApproach()
    {
        this.edges.clear();

        double smallestDistance = Double.MAX_VALUE;
        for(int i = 0; i < numOfPoints; ++i) {
            Set<Edge> tempEdges = new LinkedHashSet<>();
            double length = bruteForceApproachInt(i, new LinkedList<>(vertices), tempEdges);

            if(length < smallestDistance) {
                smallestDistance = length;
                edges = tempEdges;
            }
        }

        return smallestDistance;
    }

    /**
     * Brute force internal helper method which runs through the base case of a given Vertices index. The base case will
     * always join Vn - 1 and Vn + 1 together to form the first interior edge within the tessellation possibility.
     *
     * From this initial interior edge, two new points (Vn - 1) and (Vn + 1) are used for the internal brute force recursive
     * method which will return the shortest calculated distances starting from either one of those two points.
     *
     * The shortest will be returned and the edges set will be populated only with the shortest path using pass-by-reference.
     *
     * The list of vertices (remaining) is used to keep track of vertices which any given point can join with to form a
     * new interior edge. This list also ensures that interior edges do not cross-over one another. This list is cloned
     * for each new recursive call to prevent one reference of the list being modified in each individual tessellation
     * possibility.
     *
     * @param index of the base case e.g. one exterior Vertex's index
     * @param remaining list of vertices which are available to form Edges with
     * @param edges final set of Edges which form the shortest path
     * @return calculated shortest sum of interior edges from the shortest path found using starting point (index)
     */
    private double bruteForceApproachInt(int index, List<Vertex> remaining, Set<Edge> edges)
    {
        if(remaining.size() <= 3) // 3-sided polygons a.k.a Triangles are already a solution
        {
            return 0;
        }

        int left = boundDecrement(index, remaining.size() - 1);
        int right = boundIncrement(index, remaining.size() - 1);

        Vertex leftVertex = remaining.get(left);
        Vertex rightVertex = remaining.get(right);

        // add initial starting edge
        edges.add(new Edge(leftVertex, rightVertex));
        remaining.remove(index);

        // pass the vertex reference as that will be used to find the new index in the remaining vertices list
        List<Vertex> remainingLeft = new LinkedList<>(remaining);
        List<Vertex> remainingRight = new LinkedList<>(remaining);

        Set<Edge> leftSet = new LinkedHashSet<>(edges);
        Set<Edge> rightSet = new LinkedHashSet<>(edges);

        double bruteLeft = bruteForce(leftVertex, remainingLeft, leftSet);
        double bruteRight = bruteForce(rightVertex, remainingRight, rightSet);

        if(bruteLeft < bruteRight)
        {
            edges.addAll(leftSet);
            return bruteLeft + leftVertex.distance(rightVertex);
        }
        else
        {
            edges.addAll(rightSet);
            return bruteRight + leftVertex.distance(rightVertex);
        }
    }

    /**
     * Brute force (Top-down) internal recursive method which calculates the next possible vertices which a given Vertex can join with
     * to form non-cross-over interior edges. Typically, a given Vertex has 4 possibilities at most which it can join itself
     * with, these being currentVertex <-> (currentVertex - 2) or currentVertex <-> (currentVertex + 2). From either edge possibility,
     * the bruteForce recursive method can now be called from 4 new starting vertices:
     *
     * Left:
     *      - currentVertex
     *      - (currentVertex - 2)
     *
     * Right:
     *      - currentVertex
     *      - (currentVertex + 2)
     *
     * From these 4 possibilities, the calculated distance between either edge currentVertex <-> (currentVertex - 2) or
     * currentVertex <-> (currentVertex + 2) is added with the result from one of the four possibilities to form a singular
     * tessellation and edge set. This is because a path can restart at currentVertex since it is still in the remaining list.
     *
     * An example recursion tree diagram:
     *
     *  N = currentVertex
     *                        N
     *                   /        \
     *            (N - 2)         (N + 2)
     *            /     \         /     \
     *          (N)    (N - 2)  (N)    (N + 2)
     *
     *
     * Before progressing down the recursion tree, a singular Vertex is always removed from the remaining set, this Vertex
     * is different between Left and Right so a clone of 'remaining' is made for each branch with a different Vertex removed.
     *
     * @param currentVertex which the recursive method starts from
     * @param remaining list of vertices which are available to form an edge with
     * @param edges set of edges which keep track of the current tessellation possibility
     * @return shortest sum of interior edges
     */
    private double bruteForce(Vertex currentVertex, List<Vertex> remaining, Set<Edge> edges)
    {
        if(remaining.size() <= 3) // 3-sided polygons a.k.a Triangles are already a solution
        {
            return 0;
        }

        // get the index of the currentVertex relative to the remaining list of Vertices
        int currentIndex = remaining.indexOf(currentVertex);

        // calculate the next available left and right Vertex index located in the remaining list
        int left = boundDecrement(currentIndex, remaining.size() - 1);
        int right = boundIncrement(currentIndex, remaining.size() - 1);


        // initialise a double which keeps track of the shortest path's distance
        double chosenDistance = Double.MAX_VALUE;

        // have to get left sides left adjacent
        int leftOne = boundDecrement(left, remaining.size() - 1);

        // create two clones of the edge set, one for each possibility starting from the left index
        Set<Edge> edgeSetLeftCurrent = new LinkedHashSet<>(edges);
        Set<Edge> edgeSetLeftLeft = new LinkedHashSet<>(edges);

        // create the new interior edge
        Edge edge = new Edge(currentVertex, remaining.get(leftOne));

        // add the newly created interior edge to both copies of the cloned edge set
        edgeSetLeftCurrent.add(edge);
        edgeSetLeftLeft.add(edge);

        // create two clones of the remaining list, one for each possibility starting from the left index
        List<Vertex> remainingLeftCurrent = new LinkedList<>(remaining);
        List<Vertex> remainingLeft = new LinkedList<>(remaining);

        // remove vertex between currentVertex and remaining.get(leftOne) from both cloned remaining lists
        remainingLeftCurrent.remove(left);
        remainingLeft.remove(left);

        // calculate the shortest distance and path from both the left possibilities while also adding the distance between
        // the vertices which formed the newly created edge. This process builds up a solution.
        double leftCurrent = bruteForce(currentVertex, remainingLeftCurrent, edgeSetLeftCurrent) + currentVertex.distance(remaining.get(leftOne));
        double leftLeft = bruteForce(remaining.get(leftOne), remainingLeft, edgeSetLeftLeft) + currentVertex.distance(remaining.get(leftOne));

        // have to get right sides right adjacent
        int rightOne = boundIncrement(right, remaining.size() - 1);

        // create two clones of the edge set, one for each possibility starting from the right index
        Set<Edge> edgeSetRightCurrent = new LinkedHashSet<>(edges);
        Set<Edge> edgeSetRightRight = new LinkedHashSet<>(edges);

        // create the new interior edge
        edge = new Edge(currentVertex, remaining.get(rightOne));

        // add the newly created interior edge to both copies of the cloned edge set
        edgeSetRightCurrent.add(edge);
        edgeSetRightRight.add(edge);

        // create two clones of the remaining list, one for each possibility starting from the right index
        List<Vertex> remainingRightCurrent = new LinkedList<>(remaining);
        List<Vertex> remainingRight = new LinkedList<>(remaining);

        // remove vertex between currentVertex and remaining.get(rightOne)
        remainingRightCurrent.remove(right);
        remainingRight.remove(right);

        // calculate the shortest distance and path from both the right possibilities while also adding the distance between
        // the vertices which formed the newly created edge. This process builds up a solution.
        double rightCurrent = bruteForce(currentVertex, remainingRightCurrent, edgeSetRightCurrent) + currentVertex.distance(remaining.get(rightOne));
        double rightRight = bruteForce(remaining.get(rightOne), remainingRight, edgeSetRightRight) + currentVertex.distance(remaining.get(rightOne));

        // find the shortest path by finding the minimum distance between the four possibilities
        chosenDistance = Double.min(chosenDistance, Double.min( Double.min(leftCurrent, leftLeft), Double.min(rightCurrent, rightRight) ) );

        // this if statement boy will determine which one out of the four possibilities should have their cloned edges
        // set merged with the final copy.
        if(chosenDistance == leftCurrent)
        {
            edges.addAll(edgeSetLeftCurrent);
        }
        else if(chosenDistance == leftLeft)
        {
            edges.addAll(edgeSetLeftLeft);
        }
        else if(chosenDistance == rightCurrent)
        {
            edges.addAll(edgeSetRightCurrent);
        }
        else if(chosenDistance == rightRight)
        {
            edges.addAll(edgeSetRightRight);
        }

        // shortest path's distance
        return chosenDistance;
    }

    /**
     * Greedy approach call method which prepares the edges set, and calls the internal approach method.
     * @return approximate shortest sum of interior edges
     */
    public double greedyApproach()
    {
        this.edges.clear();
        return greedyApproachInt(new LinkedList<>(vertices), this.edges);
    }

    /**
     * Greedy approach internal method which is given a set of vertices remaining and a reference to the Polygon's shortest
     * path edge set.
     *
     * This Greedy implementation will always choose the locally optimal (shortest) path which it hopes will also form the
     * globally optimal solution. Being similar to brute force,
     *
     * The first step for the algorithm is to find the locally optimal starting Vertex, this is done by joining Vn - 1
     * and Vn + 1 together as an edge and using that distance to compare with other starting Vertices.
     *
     * From the optimal starting Vertex, the greedy algorithm will calculate the possible paths from Vn - 1 and Vn + 1,
     * and out of the maximum four possibilities, this implementation only continues along the path of the shortest one,
     * discarding the other 3 possibilities.
     *
     * This results in a highly efficient algorithm capable of 100000+ Vertices, however, since the Tessellation problem
     * does not have both optimal substructure and a greedy-choice property which results in globally optimal choices,
     * this algorithm does not always find the shortest sum of interior edges. Therefore, this algorithm is classified as
     * an approximate.
     *
     * @param vertices which make up the Polygon
     * @param edges which form the approximate shortest path
     * @return the approximate shortest sum of interior edges
     */
    private double greedyApproachInt(List<Vertex> vertices, Set<Edge> edges)
    {
        int n = vertices.size();
        if(n <= 3) // 3-sided polygons a.k.a Triangles are already a solution
        {
            return 0;
        }

        double total = 0;
        Edge start = null;

        double bestStart = Double.MAX_VALUE;
        int bestIndex = 0;

        // find the best starting point
        for(int index = 0; index < n; ++index)
        {
            int leftAdjacent = boundDecrement(index, vertices.size() - 1);
            int rightAdjacent = boundIncrement(index, vertices.size() - 1);

            double distance = vertices.get(leftAdjacent).distance( vertices.get(rightAdjacent) );

            if(distance < bestStart)
            {
                bestStart = distance;
                start = new Edge(vertices.get(leftAdjacent), vertices.get(rightAdjacent));
                bestIndex = index;
            }
        }

        // add first edge to total
        total += bestStart;
        edges.add(start);

        // remove the starting Vertex from the remaining vertices
        vertices.remove(bestIndex);

        int v1;
        int v2;

        while(vertices.size() != 3)
        {
            v1 = vertices.indexOf(start.getV1());
            v2 = vertices.indexOf(start.getV2());

            int v1leftAdj = boundDecrement(v1, vertices.size() - 1);
            int v1rightAdj = boundIncrement(v1, vertices.size() - 1);

            int v1LeftOne = boundDecrement(v1leftAdj, vertices.size() - 1);
            int v1RightOne = boundIncrement(v1rightAdj, vertices.size() - 1);

            int v2leftAdj = boundDecrement(v2, vertices.size() - 1);
            int v2rightAdj = boundIncrement(v2, vertices.size() - 1);

            int v2LeftOne = boundDecrement(v2leftAdj, vertices.size() - 1);
            int v2RightOne = boundIncrement(v2rightAdj, vertices.size() - 1);

            // the four horseman (possibilities)
            double v1Left = vertices.get(v1).distance(vertices.get(v1LeftOne));
            double v1Right = vertices.get(v1).distance(vertices.get(v1RightOne));
            double v2Left = vertices.get(v2).distance(vertices.get(v2LeftOne));
            double v2Right = vertices.get(v2).distance(vertices.get(v2RightOne));

            double chosenDistance = Double.min( Double.min(v1Left, v1Right), Double.min(v2Left, v2Right) );

            // only continue the algorithm with the locally shortest calculated possibility
            if(chosenDistance == v1Left)
            {
                Edge edge = new Edge(vertices.get(v1), vertices.get(v1LeftOne));
                start = edge;
                edges.add(edge);
                vertices.remove(v1leftAdj);
            }
            else if(chosenDistance == v1Right)
            {
                Edge edge = new Edge(vertices.get(v1), vertices.get(v1RightOne));
                start = edge;
                edges.add(edge);
                vertices.remove(v1rightAdj);
            }
            else if(chosenDistance == v2Left)
            {
                Edge edge = new Edge(vertices.get(v2), vertices.get(v2LeftOne));
                start = edge;
                edges.add(edge);
                vertices.remove(v2leftAdj);
            }
            else if(chosenDistance == v2Right)
            {
                Edge edge = new Edge(vertices.get(v2), vertices.get(v2RightOne));
                start = edge;
                edges.add(edge);
                vertices.remove(v2rightAdj);
            }

            // add the locally shortest to the now total
            total += chosenDistance;
        }

        // approximate shortest sum of interior edges
        return total;
    }

    /**
     * Exact approach (DP) call method. This method initialises the internal Exact approach which calculates the shortest
     * sum of interior edges using a Dynamic Programming Bottom-Up approach.
     * @return shortest sum of interior edges
     */
    public double exactApproach()
    {
        this.edges.clear();
        return exactApproachInt(this.vertices, this.edges);
    }

    /**
     * Exact approach internal method which utilises a Bottom-Up approach. The base case n <= 3 will return 0.
     * A 2-dimensional table of TableElement objects with size N is used to build up a table of sub-solutions.
     *
     * The outer diagonal loop iterates through all starting vertices and calculates the distance between each adjacent
     * vertex (Vn - 1) and (Vn + 1). This distance is then stored at the corresponding table index.
     *
     * From the 2nd loop, the goal is to calculate the next optimal solution using indexes I and K & K and J.
     *
     * The follow rules were following to build up the table using the 3rd loop:
     *
     * - Find the shortest distance between
     *      1 - table[i + 1][j]
     *      2 - table[i][j - 1]
     *      3 - table[i][i] + table[i + 2][j]
     *      4 - table[i - 2] + table[table.length - 1][table.length - 1]
     *
     * So from each table[i][j], four checks are being performed and the shortest out of the four is chosen, and the distance
     * between Vertex I and Vertex J is added to the chosen distance. This new value is stored at table[i][j].
     *
     * The value K for the shortest distance is stored with the shortest distance at TableElement[i][j] which is later used
     * to perform a binary search of the table[][] when creating a set of edges for the shortest path.
     *
     * The complexity is n^3 since three for loops of n size are being iterated through during this algorithm.
     *
     * @param vertices which make up the Polygon
     * @param edges set of edges which make up the shortest tessellation path
     * @return shortest sum of interior edges
     */
    private double exactApproachInt(List<Vertex> vertices, Set<Edge> edges)
    {
        if(vertices.size() <= 3) // 3-sided polygons a.k.a Triangles are already a solution
        {
            return 0;
        }

        TableElement[][] table = new TableElement[vertices.size()][vertices.size()];

        // outer diagonal
        for (int start = 0; start < vertices.size(); start++)
        {
            // inner diagonal
            for (int i = 0, j = start; j < vertices.size(); i++, j++)
            {
                // check if the current table[i][j] needs initialising and ensures that we do not form an edge with
                // adjacent vertices (Vn and Vn + 1) or (Vn - 1 and Vn).
                if ((i + 2) > j)
                {
                    table[i][j] = new TableElement(0.0, 0);
                }
                else
                {
                    double min = Double.MAX_VALUE; // initialise a blanket value which will hold the shortest distance
                    int minK = 0; // smallest K value

                    for (int k = (i + 1); k < j; k++)
                    {
                        double lowest = table[i][k].getValue() + table[k][j].getValue() + Vertex.distance(vertices.get(i), vertices.get(j));

                        // this checks whether we have reached the top-right of the table at table[0][length - 1]. if so
                        // then do not add any new calculated distances together since we will only be make an edge
                        // between the same Vertex
                        if(boundIncrement(i, vertices.size() - 1) == j | boundDecrement(i, vertices.size() - 1) == j)
                        {
                            lowest = table[i][k].getValue() + table[k][j].getValue();
                        }

                        // if the new distance is shorter than the current minimum
                        if (min > lowest)
                        {
                            min = lowest; // set the new current minimum to the newly found shortest
                            minK = k; // set the new minimum K value to current K value
                        }
                    }

                    // create a new TableElement at [i][j] with the shortest calculated distance and corresponding K value
                    table[i][j] = new TableElement(min, minK);
                }
            }
        }

        // build edges for visualisation
        buildEdges(vertices, table,0,vertices.size() - 1, edges);

        // can print table here to see the structure of my bottom-up approach.(Marker use)
        //printTable(table);

        // returns the shortest sum of interior edge distances
        return (table[0][vertices.size() - 1].getValue());
    }

    /**
     * This internal helper method builds up an Edge Set which contains only the shortest path from the Exact Approach
     * output. From the TableElement[][] list, this recursive method does a binary search through the table using the K
     * values stored within a TableElement object.
     * @param vertices list of all vertices in the polygon used to build Edge objects from indexes.
     * @param table 2-D table of TableElement objects which are used to retrieve the optimal solution using already calculated distances.
     * @param i start index e.g. 0
     * @param j end index e.g. length of vertices list - 1
     * @param edges Set of Edge objects which get filled via pass-by-reference.
     */
    private void buildEdges(List<Vertex> vertices, TableElement[][] table, int i, int j, Set<Edge> edges) {
        if((j - i) < 2) {
            return;
        }

        buildEdges(vertices, table, i, table[i][j].getK(), edges);
        if(boundIncrement(i, vertices.size() - 1) != j && boundDecrement(i, vertices.size() - 1) != j)
        {
            edges.add(new Edge(vertices.get(i), vertices.get(j)));
        }
        buildEdges(vertices, table, table[i][j].getK(), j, edges);
    }

    /**
     * This internal method prints to console the table structure used in the Exact Approach method. Mainly used for the
     * markers if they wanted to see the structure.
     * @param table 2-D table of TableElement Objects to print to console
     */
    private void printTable(TableElement[][] table)
    {
        for(int i = 0; i < vertices.size(); ++i)
        {
            for(int j = 0; j < vertices.size(); ++j)
            {
                if(table[i][j] != null)
                    System.out.print("[" + table[i][j].getValue() + "]");
                else
                    System.out.print("[   ]");
            }
            System.out.print("\n");
        }
    }

    /**
     * This object encapsulates a double value being the distance between a set of vertices, and a 'k' value which stores
     * a commonly intersected vertex.
     */
    private static class TableElement {
        private final double value; //
        private final int k; //

        /**
         * Constructor for a TableElement object given both an initial value and K value.
         * @param value distance
         * @param k intersecting vertex
         */
        public TableElement(double value, int k) {
            this.value = value;
            this.k = k;
        }

        /**
         * Get the stored distance value.
         * @return value
         */
        public double getValue() {
            return value;
        }

        /**
         * Get the stored K value.
         * @return k
         */
        public int getK() {
            return k;
        }
    }

    /**
     * This internal helper method increments a given integer in a wrap-around fashion bound to
     * a given bounded value. Similar to circular arrays.
     * @param val to increment
     * @param bound value to wrap-around
     * @return incremented value
     */
    private int boundIncrement(int val, int bound)
    {
        int value = val + 1;
        if(value > bound)
        {
            return 0;
        }
        return value;
    }

    /**
     * This internal helper method decrements a given integer in a wrap-around fashion bound to
     * a given bounded value. Similar to circular arrays.
     * @param val to decrement
     * @param bound value to wrap-around
     * @return decremented value
     */
    private int boundDecrement(int val, int bound)
    {
        int value = val - 1;
        if(value < 0)
        {
            return bound;
        }
        return value;
    }
}
