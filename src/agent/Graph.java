package agent;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Stack;
import java.awt.geom.AffineTransform;
import java.awt.geom.Ellipse2D;
import java.awt.geom.PathIterator;
import java.awt.geom.Point2D;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;

import problem.ASVConfig;
import problem.Obstacle;
import problem.ProblemSpec;
import tester.Tester;

public class Graph implements Cloneable {
	HashSet<Vertex>locations;
	HashSet<Edge> edges;
	
	public Graph(){
		locations = new HashSet<Vertex>();
		edges= new HashSet<Edge>();
	}
	
	public Graph(HashSet<Vertex> vertices,HashSet<Edge> edges, int num) {
		this.locations = vertices;
		this.edges = edges;
	}
	
	public void addLoc(Vertex loc){
		if(!locations.contains(loc)){
			locations.add(loc);
			this.edges.addAll(loc.getEdges());
		}
	}
	
	public void remLoc( Vertex loc){
		locations.remove(loc);
		this.edges.removeAll(loc.getEdges());
	}
	
	public void addE(Edge e){
		this.edges.add(e);
		this.locations.addAll(Arrays.asList(e.getV1(),e.getV2()));
	}
	
	public HashSet<Vertex> getLocations() {
		return locations;
	}
	
	public HashSet<Edge> getEdges() {
		return edges;
	}

	public int getNumberOfLocation() {
		return locations.size();
	}

	public void setLocations(HashSet<Vertex> locations) {
		this.locations = locations;
	}

	public void setEdges(HashSet<Edge> edges) {
		this.edges = edges;
	}

	public void addAllLocations(Collection<Vertex> locations) {
		this.locations.addAll(locations);
	}

	/**
	 * Helper function to retrieve a vertex by its configuration
	 * @param c the configuration wanted
	 * @return the vertex containing that configuration
	 */
	public Vertex getVertexByConfig(ASVConfig c){
		for(Vertex v: this.locations){
			if (v.getC() == c)
				return v;
		}
		return null;
	}
	@Override
	public String toString() {
		return "number of location/Edges : "+this.getNumberOfLocation()+" / "+edges.size()+"\n";
	}



}
	


