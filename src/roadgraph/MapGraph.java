/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 3
	/**
	 * Maintain both nodes and edges as you will need to
	 * be able to look up nodes by lat/lon or by roads
	 * that contain those nodes.
	 * Use HashSet to store directional edges; See MapEdge class
	 */
	private HashMap<GeographicPoint, MapNode> nodes;
	private HashSet<MapEdge> edges;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 3
		nodes = new HashMap<GeographicPoint, MapNode>();
		edges = new HashSet<MapEdge>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 3
		return nodes.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 3
		return nodes.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 3
		return edges.size();
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 3
		//if location is null, don't add it to graph
		if(location == null) return false;
		// if location is not in the graph yet, add it to the graph
		// and create an ArrayList to store its neighbors
		if(!nodes.containsKey(location)) {
			MapNode n = new MapNode(location);
			nodes.put(location, n);
			return true;
		} 
		else {
			return false;
		}
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		//TODO: Implement this method in WEEK 3
		/**
		 * throw an IllegalArgumentException if either of the two points are not in the graph already,
		 * if any of the arguments is null, or if length is less than 0.  
		 */
		MapNode n1 = nodes.get(from);
		MapNode n2 = nodes.get(to);
		
		if(n1 == null)
			throw new NullPointerException("addEdge: pt1:" + from + "is not in the graph");
		if(n2 == null)
			throw new NullPointerException("addEdge: pt2:" + from + "is not in the graph");
		
		MapEdge me = new MapEdge(roadName, roadType, n1, n2, length);
		edges.add(me);
		n1.addEdge(me);
	}
	
	/** 
	 * Get a set of neighbor nodes from a mapNode
	 * @param node  The node to get the neighbors from
	 * @return A set containing the MapNode objects that are the neighbors 
	 * 	of node
	 */
	private Set<MapNode> getNeighbors(MapNode node) {
		return node.getNeighbors();
	}
	
	/**
	 * print graph by iterating through all the nodes and print out each of its neighbors.
	 * output is not in order.
	 */
	private void printGraph() {
		for(GeographicPoint gp : nodes.keySet()) {
			MapNode n = nodes.get(gp);
			Set<MapNode> neighbors = getNeighbors(n);
			System.out.print(gp + " : ");
			for(MapNode mn : neighbors) {
				System.out.print(mn.getLocation() + ", ");
			}
			System.out.print("\n");
		}
	}

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		// create an HashMap mapping each location to its parent
		
		// Setup - check validity of inputs
		if (start == null || goal == null)
			throw new NullPointerException("Cannot find route from or to null node");
		MapNode startNode = nodes.get(start);
		MapNode endNode = nodes.get(goal);
		if (startNode == null) {
			System.err.println("Start node " + start + " does not exist");
			return null;
		}
		if (endNode == null) {
			System.err.println("End node " + goal + " does not exist");
			return null;
		}
		
		// setup to begin BFS
		HashMap<MapNode, MapNode> parentMap = new HashMap<MapNode, MapNode>();
		Queue<MapNode> toExplore = new LinkedList<MapNode>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		
		toExplore.add(startNode);
		MapNode curr = null;
		visited.add(startNode);
	
		while(!toExplore.isEmpty()) {
			System.out.println("exploring start");
			curr = toExplore.remove();
			nodeSearched.accept(curr.getLocation());
			if(curr == endNode) break;
			for(MapNode mn : getNeighbors(curr)) {
				if(!visited.contains(mn)) {
					visited.add(mn);
					parentMap.put(mn, curr);
					toExplore.add(mn);
				}
			}
		}
		// if goal is not same as curr, meaning goal not found, return null
		if(!curr.equals(endNode)) {
			System.out.println("No path found from " + start + " to " + goal);
			return null;
		}
		
		return buildPath(parentMap, startNode, endNode);
	}
	
	/**build a LinkedList of of path starting from start to end
	 * @param p HashMap mapping each GeographicPoint to its parent point
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of locations from start to end
	 * */
	private List<GeographicPoint> buildPath(HashMap<MapNode, MapNode> p,
													MapNode startNode, MapNode endNode){
		
		
		// if goal is found, find it's parent location and add it to path, until reaching the start location
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		MapNode curr = endNode;
		
		while (curr != startNode) {
			path.addFirst(curr.getLocation());
			curr = p.get(curr);
		}
		
		path.addFirst(startNode.getLocation());
		return path;
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm with time duration as factor
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstraTime(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		
		/*
		 * check if start and goal exist in graph
		 */
		MapNode sNode = nodes.get(start);
		MapNode eNode = nodes.get(goal);
		if(sNode == null) {
			throw new NullPointerException("start does not exist in graph");
		}
		if(eNode == null) {
			throw new NullPointerException("goal does not exist in graph");
		}
		
		/*set up for search*/
		PriorityQueue<MapNode> nodePQ = new PriorityQueue<MapNode>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		HashMap<MapNode, MapNode> parentMap = new HashMap<>();
		for(GeographicPoint gp : nodes.keySet()) {
			nodes.get(gp).setDist(Double.MAX_VALUE);
		}
		
		sNode.setDist(0);
		nodePQ.add(sNode);
		MapNode curr = null;
		while(!nodePQ.isEmpty()) {
			curr = nodePQ.poll();
			// Hook for visualization.  See writeup.
			//nodeSearched.accept(next.getLocation());
			nodeSearched.accept(curr.getLocation());
			if(!visited.contains(curr)) {
				visited.add(curr);
				System.out.println("Dijkstra curr node visiting " + curr);
				if(curr == eNode) break;
				Set<MapEdge> edges = curr.getEdges();
				for(MapEdge me : edges) {
					MapNode currNB = me.getEndNode();
					if(!visited.contains(currNB)) {
						double tempDist = curr.getDist() + me.getLength();
						double time = 0;
						if(me.getRoadType().equals("residential")) {
							time = tempDist/20;
						} else {
							time = tempDist/50;
						}
						if(time < currNB.getDist()) {
							currNB.setDist(time);
							parentMap.put(currNB, curr);
							nodePQ.add(currNB);
						}
					}
				}
			}
		}
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		System.out.println("Dijkstra number of nodes visited: " + visited.size());
		return buildPath(parentMap, sNode, eNode);
	}
	
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4
		
		/*
		 * check if start and goal exist in graph
		 */
		MapNode sNode = nodes.get(start);
		MapNode eNode = nodes.get(goal);
		if(sNode == null) {
			throw new NullPointerException("start does not exist in graph");
		}
		if(eNode == null) {
			throw new NullPointerException("goal does not exist in graph");
		}
		
		/*set up for search*/
		PriorityQueue<MapNode> nodePQ = new PriorityQueue<MapNode>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		HashMap<MapNode, MapNode> parentMap = new HashMap<>();
		for(GeographicPoint gp : nodes.keySet()) {
			nodes.get(gp).setDist(Double.MAX_VALUE);
		}
		
		sNode.setDist(0);
		nodePQ.add(sNode);
		MapNode curr = null;
		while(!nodePQ.isEmpty()) {
			curr = nodePQ.poll();
			// Hook for visualization.  See writeup.
			//nodeSearched.accept(next.getLocation());
			nodeSearched.accept(curr.getLocation());
			if(!visited.contains(curr)) {
				visited.add(curr);
				System.out.println("Dijkstra curr node visiting " + curr);
				if(curr == eNode) break;
				Set<MapEdge> edges = curr.getEdges();
				for(MapEdge me : edges) {
					MapNode currNB = me.getEndNode();
					if(!visited.contains(currNB)) {
						double tempDist = curr.getDist() + me.getLength();
						if(tempDist < currNB.getDist()) {
							currNB.setDist(tempDist);
							parentMap.put(currNB, curr);
							nodePQ.add(currNB);
						}
					}
				}
			}
		}
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		System.out.println("Dijkstra number of nodes visited: " + visited.size());
		return buildPath(parentMap, sNode, eNode);
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4
		/*
		 * check if start and goal exist in graph
		 */
		MapNode sNode = nodes.get(start);
		MapNode eNode = nodes.get(goal);
		if(sNode == null) {
			throw new NullPointerException("start does not exist in graph");
		}
		if(eNode == null) {
			throw new NullPointerException("goal does not exist in graph");
		}
		
		/*set up for search*/
		PriorityQueue<MapNode> nodePQ = new PriorityQueue<MapNode>(new AStarComparator());
		
		HashSet<MapNode> visited = new HashSet<MapNode>();
		HashMap<MapNode, MapNode> parentMap = new HashMap<>();
		for(GeographicPoint gp : nodes.keySet()) {
			nodes.get(gp).setDist(Double.MAX_VALUE);
			nodes.get(gp).setEstDist(Double.MAX_VALUE);
		}
		
		sNode.setDist(0);
		sNode.setEstDist(0);
		nodePQ.add(sNode);
		MapNode curr = null;
		while(!nodePQ.isEmpty()) {
			curr = nodePQ.poll();
			nodeSearched.accept(curr.getLocation());
			if(!visited.contains(curr)) {
				visited.add(curr);
				System.out.println("A star visiting " + curr);
				if(curr == eNode) break;
				Set<MapEdge> edges = curr.getEdges();
				for(MapEdge me : edges) {
					MapNode currNB = me.getEndNode();
					if(!visited.contains(currNB)) {
						double tempDist = curr.getDist() + me.getLength();
						if(tempDist < currNB.getDist()) {
							currNB.setDist(tempDist);
							currNB.setEstDist(tempDist + currNB.getLocation().distance(goal));
							parentMap.put(currNB, curr);
							nodePQ.add(currNB);
						}
					}
				}
			}
		}
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		System.out.println("A star number of nodes visited: " + visited.size());
		return buildPath(parentMap, sNode, eNode);
	}

	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		simpleTestMap.printGraph();
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		
		List<GeographicPoint> path = simpleTestMap.bfs(testStart, testEnd);
		System.out.println("printing path bfs");
		for(GeographicPoint gp : path) {
			System.out.print(gp + " ");
		}
		
		System.out.println("\n");
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		System.out.println("\n");
		System.out.println("printing path dijkstra");
		for(GeographicPoint gp : testroute) {
			System.out.print(gp + " ");
		}
		System.out.println("\n");
		
		System.out.println("printing path Astar");
		for(GeographicPoint gp : testroute2) {
			System.out.print(gp + " ");
		}
		System.out.println("\n");
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		System.out.println("printing path dijkstra");
		for(GeographicPoint gp : testroute) {
			System.out.print(gp + " ");
		}
		System.out.println("\n");
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		System.out.println("printing path dijkstra");
		for(GeographicPoint gp : testroute) {
			System.out.print(gp + " ");
		}
		System.out.println("\n");
		
		/* Use this code in Week 3 End of Week Quiz */
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		
		
	}
	
}

