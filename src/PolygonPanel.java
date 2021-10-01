import javax.swing.*;
import java.awt.*;
import java.awt.geom.Line2D;

/**
 * A PolygonPanel is an extended JPanel object used to visualise a Polygon and calculated tessellations on its PaintComponent.
 *
 * @author Waldo Theron 18033655
 */
public class PolygonPanel extends JPanel {
    private final Polygon polygon;

    private boolean showEdgeDistances = true;
    private boolean showVertexLabels = true;

    /**
     * Constructor for a PolygonPanel which requires a Polygon object as a parameter. This Polygon reference will be used
     * to retrieve the List of Vertices, Set of Edges, and Path2D of the Polygon.
     * @param polygon reference of Polygon instance
     */
    PolygonPanel(Polygon polygon)
    {
        this.polygon = polygon;
    }

    /**
     * Sets the flag showEdgeDistance to a given boolean
     * @param showEdgeDistances new flag value (true or false)
     */
    public void setShowEdgeDistances(boolean showEdgeDistances) {
        this.showEdgeDistances = showEdgeDistances;
        repaint(); // redraw panel with updated flag
    }

    /**
     * Sets the flag showVertexLabels to a given boolean
     * @param showVertexLabels new flag value (true or false)
     */
    public void setShowVertexLabels(boolean showVertexLabels) {
        this.showVertexLabels = showVertexLabels;
        repaint(); // redraw panel with updated flag
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);

        Graphics2D g2 = (Graphics2D) g;

        g2.setColor(Color.BLACK);
        g2.draw(polygon.getPolygonPath());
        g2.setFont(new Font("Arial", Font.PLAIN, 10));

        // draw each Vertex label as RED
        for(int i = 0; i < polygon.getVertices().size(); ++i)
        {
            g2.setColor(Color.RED);
            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

            if(showVertexLabels)
                g2.drawString("v"+i,(float)polygon.getVertices().get(i).getX(),(float)polygon.getVertices().get(i).getY());
        }

        // draw each interior Edge as BLUE lines
        for(Edge edge : polygon.getEdges())
        {
            Line2D lineEdge = new Line2D.Double(edge.getV1().getX(), edge.getV1().getY(), edge.getV2().getX(), edge.getV2().getY());
            g2.setColor(Color.BLUE);
            g2.draw(lineEdge);
            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

            // was gonna be used to rotate text along interior edges but thought rather not
            //double theta = Math.tanh( (edge.getV2().getY() - edge.getV1().getY()) / (edge.getV2().getX() - edge.getV1().getX()) );

            if(showEdgeDistances)
                g2.drawString(edge.getV1().distance(edge.getV2()) + "",(float)((edge.getV1().getX() + edge.getV2().getX()) / 2),(float)((edge.getV1().getY() + edge.getV2().getY()) / 2));
        }
    }
}