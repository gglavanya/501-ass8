import java.util.*;
import javafx.animation.KeyFrame;
import javafx.animation.Timeline;
import javafx.application.Application;
import javafx.scene.Scene;
import javafx.scene.control.ScrollPane;
import javafx.scene.control.Tab;
import javafx.scene.control.TabPane;
import javafx.scene.control.Tooltip;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Line;
import javafx.scene.text.Text;
import javafx.stage.Stage;
import javafx.util.Duration;

class NodePoint {
    int node_number;
    double x, y;
    int packet;

    public NodePoint(int node_number, double x, double y) {
        this.node_number = node_number;
        this.x = x;
        this.y = y;
        // Random packet count between < 100
        this.packet = new Random().nextInt(100) + 1; 
    }
}

class Edge {
    NodePoint destination;
    double weight;

    public Edge(NodePoint destination, double weight) {
        this.destination = destination;
        this.weight = weight;
    }
}

class SensorGraph {
    int num_nodes;
    double tr_range, width, height;
    NodePoint[] nodes;
    List<List<Edge>> adj_list;
    double[][] adj_matrix;
    Map<NodePoint, NodePoint> next_pt_map;
    Map<NodePoint, Map<NodePoint, NodePoint>> pre_comp_paths;
    Map<NodePoint, List<Edge>> mst_comps;
    double total_min_energy = 0;
    double total_mst_energy = 0;

    List<Line> cur_paths = new ArrayList<>();

    public SensorGraph(int num_nodes, double tr_range, double width, double height, int min_packets, int max_packets) {
        this.num_nodes = num_nodes;
        this.width = width;
        this.height = height;
        this.nodes = new NodePoint[num_nodes];
        this.tr_range = tr_range;
        this.genNodes(width, height);
        this.adj_matrix = new double[num_nodes][num_nodes];
        this.adj_list = this.buildAdjacencyList(num_nodes);
        this.next_pt_map = assignDestPoints();
        this.pre_comp_paths = precomputeDestPaths();
        this.mst_comps = calculateComponentMSTs();
    }

    public void genNodes(double width, double height) {
        for (int i = 0; i < num_nodes; i++) {
            Random rand = new Random();
            nodes[i] = new NodePoint(i, width * rand.nextDouble() + 20, height * rand.nextDouble() + 20);
        }
    }

    class EdgeMst {
        NodePoint start;
        NodePoint end;
        double weight;
        
        public EdgeMst(NodePoint start, NodePoint end) {
            this.start = start;
            this.end = end;
            this.weight = calculateEuclideanDistance(start, end);
        }
        static double calculateEuclideanDistance(NodePoint src, NodePoint dest) {
            return Math.sqrt(Math.pow(src.x - dest.x, 2) + Math.pow(src.y - dest.y, 2));
        }
    }

    List<List<Edge>> buildAdjacencyList(int num_nodes) {
        List<List<Edge>> adj_list = new ArrayList<>();
        for (int i = 0; i < num_nodes; i++) {
            List<Edge> neighbor_edges_list = new LinkedList<>();
            for (int j = 0; j < num_nodes; j++) {
                double distance = calculateEuclideanDistance(this.nodes[i], this.nodes[j]);
                if (i != j && distance <= tr_range) {
                    double weight = calculateTransmissionEnergy(distance);
                    neighbor_edges_list.add(new Edge(nodes[j], weight));
                    this.adj_matrix[i][j] = weight;
                }}
            adj_list.add(i, neighbor_edges_list);
        }
        return adj_list;
    }

    static double calculateTransmissionEnergy(double distance) {
        final double x = 100e-6;
        final double y = 100e-9;
        return 3200 * (y * Math.pow(distance, 2) + x);
    }

    static double calculateEuclideanDistance(NodePoint src, NodePoint dest) {
        return Math.sqrt(Math.pow(src.x - dest.x, 2) + Math.pow(src.y - dest.y, 2));
    }

    public List<List<Integer>> breadthFirstSearchMatrix() {
        boolean[] visited = new boolean[num_nodes];
        List<List<Integer>> result = new ArrayList<>();
        Arrays.fill(visited, false);
        
        for (int i = 0; i < num_nodes; i++) {
            Queue<NodePoint> queue = new LinkedList<>();
            if (!visited[i]) {
                List<Integer> node_bfs_list = new ArrayList<>();
                queue.add(nodes[i]);
                visited[i] = true;
                node_bfs_list.add(nodes[i].node_number + 1);
                
                while (!queue.isEmpty()) {
                    NodePoint current = queue.poll();
                    for (int j = 0; j < num_nodes; j++) {
                        if (j != current.node_number && adj_matrix[current.node_number][j] != 0 && !visited[j]) {
                            node_bfs_list.add(j + 1);
                            visited[j] = true;
                            queue.add(nodes[j]);
                        }}}
                result.add(node_bfs_list);
            }
        }
        return result;
    }

    public List<List<Integer>> depthFirstSearchMatrix() {
        List<List<Integer>> result = new ArrayList<>();
        boolean[] visited = new boolean[num_nodes];
        
        for (int i = 0; i < num_nodes; i++) {
            if (!visited[i]) {
                List<Integer> node_dfs_list = new ArrayList<>();
                dfsMatrixRec(visited, node_dfs_list, i);
                result.add(node_dfs_list);
            }}
        return result;
    }

    public void dfsMatrixRec(boolean[] visited, List<Integer> node_dfs_list, int node) {
        visited[node] = true;
        node_dfs_list.add(node + 1);
        
        for (int i = 0; i < num_nodes; i++) {
            if (!visited[i] && adj_matrix[node][i] != 0) {
                dfsMatrixRec(visited, node_dfs_list, i);
            }}
    }

    public List<List<Integer>> breadthFirstSearchList() {
        Queue<NodePoint> queue = new LinkedList<>();
        List<List<Integer>> result = new ArrayList<>();
        boolean[] visited = new boolean[num_nodes];
        Arrays.fill(visited, false);
        
        for (int i = 0; i < num_nodes; i++) {
            if (!visited[i]) {
                List<Integer> node_bfs_list = new ArrayList<>();
                queue.add(nodes[i]);
                visited[i] = true;
                node_bfs_list.add(nodes[i].node_number + 1);

                while (!queue.isEmpty()) {
                    NodePoint current = queue.poll();
                    List<Edge> neighbours = adj_list.get(current.node_number);
                    
                    while (!neighbours.isEmpty()) {
                        NodePoint node_point = neighbours.removeFirst().destination;
                        if (!visited[node_point.node_number]) {
                            node_bfs_list.add(node_point.node_number + 1);
                            visited[node_point.node_number] = true;
                            queue.add(node_point);
                        }}}
                        result.add(node_bfs_list);
            }
        }
        return result;
    }

    public List<List<Integer>> depthFirstSearchList() {
        List<List<Integer>> result = new ArrayList<>();
        boolean[] visited = new boolean[num_nodes];
        
        for (int i = 0; i < num_nodes; i++) {
            if (!visited[i]) {
                List<Integer> node_dfs_list = new ArrayList<>();
                dfsListRec(visited, node_dfs_list, i);
                result.add(node_dfs_list);
            }}
            return result;
    }

    private void dfsListRec(boolean[] visited, List<Integer> node_dfs_list, int node) {
        visited[node] = true;
        node_dfs_list.add(node + 1);
        List<Edge> neighbours = adj_list.get(node);
        
        while (!neighbours.isEmpty()) {
            Edge node_point = neighbours.removeFirst();
            if (!visited[node_point.destination.node_number]) {
                dfsListRec(visited, node_dfs_list, node_point.destination.node_number);
            }}
    }

    public Map<NodePoint, NodePoint> assignDestPoints() {
        Map<NodePoint, NodePoint> destination_map = new HashMap<>();
        boolean[] visited = new boolean[num_nodes];

        for (int i = 0; i < num_nodes; i++) {
            if (!visited[i]) {
                List<NodePoint> component = new ArrayList<>();
                findConnectedComponent(i, visited, component);

                if (!component.isEmpty()) {
                    NodePoint destination_point = component.get(new Random().nextInt(component.size()));
                    for (NodePoint node : component) {
                        destination_map.put(node, destination_point);
                    }}}}
        return destination_map;
    }

    private void findConnectedComponent(int start, boolean[] visited, List<NodePoint> component) {
        Queue<NodePoint> queue = new LinkedList<>();
        queue.add(nodes[start]);
        visited[start] = true;

        while (!queue.isEmpty()) {
            NodePoint current = queue.poll();
            component.add(current);

            for (Edge edge : adj_list.get(current.node_number)) {
                if (!visited[edge.destination.node_number]) {
                    visited[edge.destination.node_number] = true;
                    queue.add(edge.destination);
                }}}
            }

    private List<Edge> computePrimMST(NodePoint start_node) {
        List<Edge> mst_edges = new ArrayList<>();
        Set<NodePoint> visited = new HashSet<>();
        PriorityQueue<Edge> edge_queue = new PriorityQueue<>(Comparator.comparing(e -> e.weight));

        visited.add(start_node);
        edge_queue.addAll(adj_list.get(start_node.node_number));

        while (!edge_queue.isEmpty()) {
            Edge edge = edge_queue.poll();
            NodePoint dest = edge.destination;
            
            if (!visited.contains(dest)) {
                mst_edges.add(edge);
                visited.add(dest);

                for (Edge next_edge : adj_list.get(dest.node_number)) {
                    if (!visited.contains(next_edge.destination)) {
                        edge_queue.add(next_edge);
                    }}}
                }
        return mst_edges;
    }

    public List<Edge> buildMeshNetwork(Set<NodePoint> destination_points, Pane pane) {
        List<Edge> mesh_edges = new ArrayList<>();
        List<NodePoint> points = new ArrayList<>(destination_points);

        for (int i = 0; i < points.size(); i++) {
            for (int j = i + 1; j < points.size(); j++) {
                NodePoint point_a = points.get(i);
                NodePoint point_b = points.get(j);
                double distance = calculateEuclideanDistance(point_a, point_b);
                double energy = calculateTransmissionEnergy(distance);

                mesh_edges.add(new Edge(point_b, energy));
                mesh_edges.add(new Edge(point_a, energy));
                Line line = new Line(point_a.x, point_a.y, point_b.x, point_b.y);
                line.setStroke(Color.GREEN);
                line.setStrokeWidth(2);
                pane.getChildren().add(line);
            }}
            return mesh_edges;
    }

    public void renderMeshNetwork(Pane pane) {
        Set<NodePoint> destination_points = new HashSet<>(next_pt_map.values());
        destination_points.add(new NodePoint(-1, 10, 10));
        for (NodePoint node : destination_points) {
            Circle circle = new Circle(node.x, node.y, 7, Color.BLUE);
            Text text = new Text(node.x, node.y, "R" + (node.node_number + 1));
            Tooltip tooltip = new Tooltip("Dest Point " + (node.node_number + 1));
            Tooltip.install(circle, tooltip);
            pane.getChildren().addAll(circle, text);
        }
    }

    private void renderDestPoints(Pane pane, Set<NodePoint> destination_points) {
        for (NodePoint node : destination_points) {
            Circle circle = new Circle(node.x, node.y, 7, Color.RED);
            Text text = new Text(node.x, node.y, "R" + (node.node_number + 1));
            Tooltip tooltip = new Tooltip("Dest Point " + (node.node_number + 1));
            Tooltip.install(circle, tooltip);
            pane.getChildren().addAll(circle, text);
        }
    }

    private void drawPathConnections(Pane pane, List<NodePoint> tsp_solution) {
        for (int i = 0; i < tsp_solution.size() - 1; i++) {
            NodePoint from = tsp_solution.get(i);
            NodePoint to = tsp_solution.get(i + 1);

            Line line = new Line(from.x, from.y, to.x, to.y);
            line.setStroke(Color.GREEN);
            line.setStrokeWidth(2);
            pane.getChildren().add(line);
        }
    }

    public List<NodePoint> computeGreedyTSP(Set<NodePoint> destination_points, Pane pane) {
        NodePoint depot = new NodePoint(-1, 10, 10);
        List<NodePoint> tsp_solution = new ArrayList<>();
        Set<NodePoint> points_to_visit = new HashSet<>(destination_points);
        NodePoint selected_node = depot;
        tsp_solution.add(selected_node);

        while (!points_to_visit.isEmpty()) {
            double min_distance = Double.MAX_VALUE;
            NodePoint nearest_node = null;

            for (NodePoint node : points_to_visit) {
                double curr_distance = calculateEuclideanDistance(selected_node, node);
                if (curr_distance < min_distance) {
                    min_distance = curr_distance;
                    nearest_node = node;
                }}
                tsp_solution.add(nearest_node);
            points_to_visit.remove(nearest_node);
            selected_node = nearest_node;
        }

        tsp_solution.add(depot);
        drawPathConnections(pane, tsp_solution);
        return tsp_solution;
    }

    public void renderGreedyTSP(Pane pane) {
        Set<NodePoint> destination_points = new HashSet<>(next_pt_map.values());
        destination_points.add(new NodePoint(-1, 10, 10));
        renderDestPoints(pane, destination_points);
        
        double total_distance = 0;
        List<NodePoint> tsp_path = computeGreedyTSP(destination_points, pane);

        for (int i = 0; i < tsp_path.size() - 1; i++) {
            NodePoint from = tsp_path.get(i);
            NodePoint to = tsp_path.get(i + 1);

            Line line = new Line(from.x, from.y, to.x, to.y);
            line.setStroke(Color.BLUE);
            line.setStrokeWidth(2);
            pane.getChildren().add(line);

            total_distance += EdgeMst.calculateEuclideanDistance(from, to);
        }

        System.out.println("The Energy from Greedy tsp: " + total_distance * 100);
    }

    private void renderAllMSTs(Pane pane) {
        for (NodePoint node : nodes) {
            Circle circle = new Circle(node.x, node.y, 5, Color.RED);
            Text text = new Text(node.x + 8, node.y, String.valueOf(node.node_number + 1));

            if (next_pt_map.get(node) == node) {
                circle.setFill(Color.BLUE);
                circle.setRadius(7);
                Text destination_text = new Text(node.x + 10, node.y, "R" + (node.node_number + 1));
                pane.getChildren().add(destination_text);
            }
            pane.getChildren().addAll(circle, text);
        }

        for (NodePoint destination : mst_comps.keySet()) {
            List<Edge> mst_edges = mst_comps.get(destination);
            if (mst_edges != null) {
                for (Edge edge : mst_edges) {
                    NodePoint start_node = destination;
                    NodePoint end_node = edge.destination;
                    List<NodePoint> path = findPathBetweenNodes(start_node, end_node);
                    drawPathConnections(pane, path);
                }}}
    }

    public void computeMSTBasedTSP(Pane pane) {
        Set<NodePoint> rendezvous_points = new HashSet<>(next_pt_map.values());
        NodePoint depot = new NodePoint(-1, 10, 10);
        rendezvous_points.add(depot);

        renderDestPoints(pane, rendezvous_points);
        Map<NodePoint, List<NodePoint>> mst = constructMinimumSpanningTree(rendezvous_points);
        List<NodePoint> tsp_path = new ArrayList<>();
        Set<NodePoint> visited = new HashSet<>();
        performPreorderTraversal(depot, mst, tsp_path, visited);
        renderTSPPath(tsp_path, pane);
    }

    private Map<NodePoint, List<NodePoint>> constructMinimumSpanningTree(Set<NodePoint> nodes) {
        Map<NodePoint, List<NodePoint>> mst = new HashMap<>();
        PriorityQueue<EdgeMst> pq = new PriorityQueue<>(Comparator.comparingDouble(e -> e.weight));

        NodePoint start = nodes.iterator().next();
        Set<NodePoint> visited = new HashSet<>();
        visited.add(start);

        for (NodePoint neighbor : nodes) {
            if (!neighbor.equals(start)) {
                pq.add(new EdgeMst(start, neighbor));
            }
        }

        while (!pq.isEmpty() && visited.size() < nodes.size()) {
            EdgeMst edge = pq.poll();
            if (visited.contains(edge.end)) continue;

            mst.computeIfAbsent(edge.start, k -> new ArrayList<>()).add(edge.end);
            mst.computeIfAbsent(edge.end, k -> new ArrayList<>()).add(edge.start);

            visited.add(edge.end);

            for (NodePoint neighbor : nodes) {
                if (!visited.contains(neighbor)) {
                    pq.add(new EdgeMst(edge.end, neighbor));
                }
            }
        }
        return mst;
    }

    private void performPreorderTraversal(NodePoint node, Map<NodePoint, List<NodePoint>> mst, List<NodePoint> tsp_path, Set<NodePoint> visited) {
        visited.add(node);
        tsp_path.add(node);

        List<NodePoint> neighbors = mst.getOrDefault(node, Collections.emptyList());

        for (NodePoint neighbor : neighbors) {
            if (!visited.contains(neighbor)) {
                performPreorderTraversal(neighbor, mst, tsp_path, visited);
            }
        }
    }

    private void renderTSPPath(List<NodePoint> tsp_path, Pane pane) {
        double total_distance = 0;

        for (int i = 0; i < tsp_path.size() - 1; i++) {
            NodePoint from = tsp_path.get(i);
            NodePoint to = tsp_path.get(i + 1);

            Line line = new Line(from.x, from.y, to.x, to.y);
            line.setStroke(Color.BLUE);
            line.setStrokeWidth(2);
            pane.getChildren().add(line);
            total_distance += EdgeMst.calculateEuclideanDistance(from, to);
        }

        NodePoint last_node = tsp_path.get(tsp_path.size() - 1);
        NodePoint depot = tsp_path.get(0);

        Line line = new Line(last_node.x, last_node.y, depot.x, depot.y);
        line.setStroke(Color.BLUE);
        line.setStrokeWidth(2);
        pane.getChildren().add(line);

        System.out.println("Approximation Energy: " + total_distance * 100);
    }

    public void visualizeNetworkGraph(Stage primary_stage) {
        TabPane tab_pane = new TabPane();

        Tab graph_tab = new Tab("Graph");
        Pane graph_pane = new Pane();
        renderNetworkGraph(graph_pane);
        renderAllMSTs(graph_pane);
        computeMSTBasedTSP(graph_pane);
        renderGreedyTSP(graph_pane);

        graph_tab.setContent(new ScrollPane(graph_pane));
        tab_pane.getTabs().add(graph_tab);

        Scene scene = new Scene(tab_pane, width + 100, height + 100);
        primary_stage.setTitle("501 HW8 - Wireless Sensor Network Visualization with MSTs");
        primary_stage.setScene(scene);
        primary_stage.show();
    }

    private void renderNetworkGraph(Pane pane) {
        for (NodePoint node : nodes) {
            Circle circle = new Circle(node.x, node.y, 5, Color.RED);
            Text text = new Text(node.x + 8, node.y, String.valueOf(node.node_number + 1));

            if (next_pt_map.get(node) == node) {
                circle.setFill(Color.BLUE);
                circle.setRadius(7);
                Text destination_text = new Text(node.x + 10, node.y, "R" + (node.node_number + 1));
                pane.getChildren().add(destination_text);
            }
            
            pane.getChildren().addAll(circle, text);
        }

        for (int i = 0; i < num_nodes; i++) {
            for (int j = i + 1; j < num_nodes; j++) {
                if (adj_matrix[i][j] != 0) {
                    Line line = new Line(nodes[i].x, nodes[i].y, nodes[j].x, nodes[j].y);
                    line.setStroke(Color.LIGHTGRAY);
                    pane.getChildren().add(line);
                }
            }
        }
    }

    private List<NodePoint> findPathBetweenNodes(NodePoint start, NodePoint end) {
        Stack<NodePoint> stack = new Stack<>();
        Map<NodePoint, NodePoint> parent_map = new HashMap<>();
        Set<NodePoint> visited = new HashSet<>();
        stack.push(start);
        visited.add(start);

        while (!stack.isEmpty()) {
            NodePoint current = stack.pop();
            if (current == end) break;

            for (Edge edge : adj_list.get(current.node_number)) {
                NodePoint neighbor = edge.destination;
                if (!visited.contains(neighbor)) {
                    visited.add(neighbor);
                    parent_map.put(neighbor, current);
                    stack.push(neighbor);
                }
            }
        }

        List<NodePoint> path = new ArrayList<>();
        for (NodePoint at = end; at != null; at = parent_map.get(at)) {
            path.add(at);
        }
        Collections.reverse(path);
        return path;
    }

    public Map<NodePoint, NodePoint> computeDijkstraShortestPath(NodePoint start_node) {
        Map<NodePoint, Double> distances = new HashMap<>();
        Map<NodePoint, NodePoint> predecessors = new HashMap<>();
        PriorityQueue<NodePoint> queue = new PriorityQueue<>(Comparator.comparing(distances::get));

        for (NodePoint node : nodes) {
            distances.put(node, Double.MAX_VALUE);
            queue.add(node);
        }
        distances.put(start_node, 0.0);

        while (!queue.isEmpty()) {
            NodePoint current = queue.poll();
            for (Edge edge : adj_list.get(current.node_number)) {
                double new_dist = distances.get(current) + edge.weight;
                if (new_dist < distances.get(edge.destination)) {
                    distances.put(edge.destination, new_dist);
                    predecessors.put(edge.destination, current);
                    queue.remove(edge.destination);
                    queue.add(edge.destination);
                }}}
                return predecessors;
    }

    public Map<NodePoint, Map<NodePoint, NodePoint>> precomputeDestPaths() {
        Map<NodePoint, Map<NodePoint, NodePoint>> paths = new HashMap<>();
        
        for (NodePoint destination : new HashSet<>(next_pt_map.values())) {
            Map<NodePoint, NodePoint> path_map = computeDijkstraShortestPath(destination);
            paths.put(destination, path_map);

            for (NodePoint node : nodes) {
                if (path_map.containsKey(node)) {
                    updatePathWithEnergy(destination, node, path_map);
                }}}
        return paths;
    }

    private void updatePathWithEnergy(NodePoint start, NodePoint end, Map<NodePoint, NodePoint> predecessors) {
        List<NodePoint> path = new ArrayList<>();
        double total_energy = 0;
        NodePoint current = end;

        while (current != null && current != start) {
            NodePoint prev = predecessors.get(current);
            if (prev == null) break;
            path.add(current);

            double distance = calculateEuclideanDistance(prev, current);
            total_energy += calculateTransmissionEnergy(distance) * start.packet;
            current = prev;
        }
        path.add(start);
        Collections.reverse(path);
        total_min_energy += total_energy;
    }

    public Map<NodePoint, List<Edge>> calculateComponentMSTs() {
        Map<NodePoint, List<Edge>> mst_map = new HashMap<>();
        Set<NodePoint> unique_destination_points = new HashSet<>(next_pt_map.values());

        for (NodePoint destination : unique_destination_points) {
            List<Edge> mst_edges = computePrimMST(destination);
            mst_map.put(destination, mst_edges);
            for (Edge edge : mst_edges) {
                total_mst_energy = total_mst_energy + (edge.weight * edge.destination.packet) * mst_edges.size();
            }
        }
        return mst_map;
    }
}

public class Main extends Application {
    static SensorGraph sensor_graph;

    public static void main(String[] args) {
        String method = "";
        int width, height, num_nodes, graph_structure, traversal_method, min_packets, max_packets;
        double tr_range;
        
        if (args.length == 8) {
            width = Integer.parseInt(args[0]);
            height = Integer.parseInt(args[1]);
            num_nodes = Integer.parseInt(args[2]);
            tr_range = Double.parseDouble(args[3]);
            graph_structure = Integer.parseInt(args[4]);
            traversal_method = Integer.parseInt(args[5]);
            min_packets = Integer.parseInt(args[6]);
            max_packets = Integer.parseInt(args[7]);
        } else {
            Scanner sc = new Scanner(System.in);
            System.out.print("Width: ");
            width = sc.nextInt();
            System.out.print("Height: ");
            height = sc.nextInt();
            System.out.print("Node Count: ");
            num_nodes = sc.nextInt();
            System.out.print("Transmission Range: ");
            tr_range = sc.nextDouble();
            System.out.print("1 = matrix, 2 = list): ");
            graph_structure = sc.nextInt();
            System.out.print("1 = BFS, 2 = DFS: ");
            traversal_method = sc.nextInt();
            System.out.print("Min Packets: ");
            min_packets = sc.nextInt();
            System.out.print("Max packets: ");
            max_packets = sc.nextInt();
            sc.close();
        }

        sensor_graph = new SensorGraph(num_nodes, tr_range, width, height, min_packets, max_packets);
        List<List<Integer>> traversal_list = List.of();
        
        if (graph_structure == 1 && traversal_method == 1) {
            traversal_list = sensor_graph.breadthFirstSearchMatrix();
            method = "BFS";
        }
        if (graph_structure == 1 && traversal_method == 2) {
            traversal_list = sensor_graph.depthFirstSearchMatrix();
            method = "DFS";
        }
        if (graph_structure == 2 && traversal_method == 1) {
            traversal_list = sensor_graph.breadthFirstSearchList();
            method = "BFS";
        }
        if (graph_structure == 2 && traversal_method == 2) {
            traversal_list = sensor_graph.depthFirstSearchList();
            method = "DFS";
        }
        
        System.out.println("The Energy from MST: " + sensor_graph.total_mst_energy);
        System.out.println("The Energy from Dijkstra path: " + sensor_graph.total_min_energy);
        launch(args);
    }

    @Override
    public void start(Stage primary_stage) {
        sensor_graph.visualizeNetworkGraph(primary_stage);
    }
}