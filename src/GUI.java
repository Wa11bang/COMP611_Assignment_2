import javax.swing.*;
import javax.swing.border.EmptyBorder;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.ItemEvent;
import java.awt.event.ItemListener;

/**
 * GUI is the main graphical interface Object used to handle user requests and interactions. A list of handled interactions
 * include:
 *
 * - Visualising Polygon Paths
 * - Visualising Polygon Tessellations
 * - Generating new Polygons between 3 and 100 vertices
 * - Calling methods which calculate Tessellations based on a Brute Force Approach
 * - Calling methods which calculate Tessellations based on an Approximate Greedy Approach
 * - Calling methods which calculate Tessellations based on an Exact (Dynamix Programming) Bottom-Up Approach
 * - Enabling/Disabling Vertex Labels
 * - Enabling/Disabling Edge Distances
 *
 * @author Waldo Theron 18033655
 */
public class GUI extends JFrame implements ActionListener, ItemListener, ChangeListener {
    private static final int DEFAULT_POLYGON_SIZE = 7;

    private static final JLabel sum = new JLabel("Calculating...");
    private static final JLabel numOfVertices = new JLabel("Vertices: 7");

    private final JButton randomPolygon = new JButton("Generate Random Polygon");

    private final JToggleButton greedyToggle = new JToggleButton("Toggle Greedy");
    private final JToggleButton bruteForceToggle = new JToggleButton("Toggle Brute Force");
    private final JToggleButton exactToggle = new JToggleButton("Toggle Exact");

    private final JSlider numOfVerticesSlider = new JSlider(JSlider.HORIZONTAL, 3, 100, 7);

    private final JCheckBox showEdgeDistances = new JCheckBox("Show Edge Distances");
    private final JCheckBox showVertexLabels = new JCheckBox("Show Vertex Labels");

    private PolygonPanel polygonPanel;
    private Polygon polygon;

    /**
     * This internal helper method generates a JOptionPane which welcomes the user to the GUI and gives a brief
     * explanation on the functionality.
     */
    private void showInitialPopupDialog()
    {
        JLabel message = new JLabel("<html><p><center>Hello, Welcome to Tessellations.<br/><br/>Please select from one of the approaches available.<br/>Polygon sizes 4 - 12 recommended for Brute Force.<br/>Polygon sizes 12 - 100 recommended for Greedy & Exact.<br/><br/>Vertex labels and edge distances can be disabled.</center></p></html>");
        JOptionPane.showMessageDialog(this, message, "Welcome", JOptionPane.PLAIN_MESSAGE);
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        if(e.getSource().equals(randomPolygon))
        {
            Polygon poly = new Polygon(numOfVerticesSlider.getValue());
            sum.setText("Select an approach");

            remove(polygonPanel);
            addPolygon(poly);

            bruteForceToggle.setSelected(false);
            greedyToggle.setSelected(false);
            exactToggle.setSelected(false);
        }
    }

    @Override
    public void itemStateChanged(ItemEvent e) {
        if(e.getSource() == showEdgeDistances)
        {
            polygonPanel.setShowEdgeDistances(showEdgeDistances.isSelected());
        }
        if(e.getSource() == showVertexLabels)
        {
            polygonPanel.setShowVertexLabels(showVertexLabels.isSelected());
        }
        if(e.getSource() == bruteForceToggle && bruteForceToggle.isSelected())
        {
            greedyToggle.setSelected(false);
            exactToggle.setSelected(false);
            sum.setText("Sum of interior edges (brute): " + polygon.bruteForceApproach());
        }
        if(e.getSource() == greedyToggle && greedyToggle.isSelected())
        {
            bruteForceToggle.setSelected(false);
            exactToggle.setSelected(false);
            sum.setText("Sum of interior edges (greedy): " + polygon.greedyApproach());
        }
        if(e.getSource() == exactToggle && exactToggle.isSelected())
        {
            bruteForceToggle.setSelected(false);
            greedyToggle.setSelected(false);
            sum.setText("Sum of interior edges (exact): " + polygon.exactApproach());
        }

        repaint(); // redraws the GUI since a new Tessellation approach has been selected
    }

    @Override
    public void stateChanged(ChangeEvent e) {
        if(e.getSource() == numOfVerticesSlider)
        {
            numOfVertices.setText("Vertices: " + numOfVerticesSlider.getValue());
        }
    }

    /**
     * Constructor for a GUI Object. Initialises all JFrame values, JComponents, and event handlers.
     *
     * A Popup dialog is shown after initialisation has completed.
     */
    public GUI()
    {
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        setVisible(true);
        setResizable(false);
        setSize(800, 825);
        setLocationRelativeTo(null);
        setTitle("Tessellations");

        sum.setFont(new Font("Arial", Font.BOLD, 20));
        numOfVertices.setFont(new Font("Arial", Font.BOLD, 15));

        showVertexLabels.setSelected(true);
        showEdgeDistances.setSelected(true);
        showVertexLabels.addItemListener(this);
        showEdgeDistances.addItemListener(this);

        JPanel topPanel = new JPanel(new BorderLayout());
        JPanel checkboxPanel = new JPanel(new FlowLayout());
        checkboxPanel.add(showVertexLabels);
        checkboxPanel.add(showEdgeDistances);

        topPanel.setBorder(new EmptyBorder(10, 10, 10, 10));
        topPanel.add(checkboxPanel, BorderLayout.LINE_END);
        topPanel.add(sum, BorderLayout.NORTH);
        topPanel.add(numOfVertices, BorderLayout.CENTER);

        add(topPanel, BorderLayout.NORTH);

        bruteForceToggle.addItemListener(this);
        greedyToggle.addItemListener(this);
        exactToggle.addItemListener(this);
        numOfVerticesSlider.addChangeListener(this);
        numOfVerticesSlider.setPaintLabels(true);
        numOfVerticesSlider.setBorder(BorderFactory.createTitledBorder("Number of Vertices"));

        JPanel bottomPanel = new JPanel(new GridBagLayout());
        bottomPanel.setBorder(new EmptyBorder(0, 0, 10, 0));
        bottomPanel.add(numOfVerticesSlider);
        bottomPanel.add(randomPolygon);
        bottomPanel.add(bruteForceToggle);
        bottomPanel.add(greedyToggle);
        bottomPanel.add(exactToggle);

        add(bottomPanel, BorderLayout.SOUTH);

        randomPolygon.addActionListener(this);

        showInitialPopupDialog();
    }

    /**
     * This method stores a reference of a given Polygon instance, creates a new PolygonPanel with said Polygon and
     * adds the newly created PolygonPanel to the GUI JFrame.
     * @param polygon reference of Polygon instance
     */
    public void addPolygon(Polygon polygon)
    {
        this.polygon = polygon;
        this.polygonPanel = new PolygonPanel(this.polygon);
        this.polygonPanel.setShowVertexLabels(showVertexLabels.isSelected());
        this.polygonPanel.setShowEdgeDistances(showEdgeDistances.isSelected());

        add(polygonPanel, BorderLayout.CENTER);

        revalidate(); // recalculates JComponents in JFrame so that repainting is done correctly.
    }

    /**
     * Main method used to start the GUI program. Initialises a default Polygon of DEFAULT_POLYGON_SIZE which is
     * subsequently added to the GUI.
     */
    public static void main(String... args)
    {
        GUI gui = new GUI();

        Polygon polygon = new Polygon(DEFAULT_POLYGON_SIZE);

        System.out.println("Adding Polygon to Panel...\n");
        sum.setText("Select an approach");

        gui.addPolygon(polygon);
    }
}