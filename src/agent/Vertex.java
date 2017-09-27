package agent;

import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Stack;

import problem.ASVConfig;

public class Vertex {
	//int id;
	HashSet<Edge> edges;
	double pathCost = Integer.MAX_VALUE;//Max value by default
	double h = -1;
	ASVConfig c;


	/**
	 * Creates a new vertex
	 */
	public Vertex(){
		//this.id =-1;
		this.edges = new HashSet<Edge>();
	}
	

	
	
	public Vertex(ASVConfig c){
		this.c = c;
		this.edges = new HashSet<Edge>();
	}

	
	public void addEdge(Edge e){
		this.edges.add(e);
	}
	
	public void setEdges(HashSet<Edge>edges){
		this.edges = edges;
	}
	
//	public boolean isValid() {
//		return isValid.booleanValue();
//	}
//
//	public boolean validIsSet(){
//		return isValid!=null;
//	}
//
//	public void setValid(boolean isValid) {
//		this.isValid = Boolean.valueOf(isValid);
//	}


	public HashSet<Edge> getEdges(){
		return this.edges;	
	}
	/**
	 * @assumes the edge e is a valid edge for v
	 * @param e
	 */
	public void addE(Edge e){
		this.edges.add(e);
	}
	
	public void setPathCost(double cost){
		this.pathCost = cost;
	}
	
	public double getPathCost(){
		return this.pathCost;
	}
	
	public double getF() {
		return pathCost + h;
	}
	
	public double getH() {
		return h;
	}
	
	public void setH(double h) {
		this.h = h;
	}
	
	public boolean intersects(Vertex v1){
		for(Edge e: edges){
			if(e.contains(v1)){
				return true;
			}
		}
		return false;
	}

	@Override
	public int hashCode() {
		final int prime = 3;
		int result = 1;
		result = prime * result + c.hashCode();
		//result = prime * result + ((edges == null) ? 0 : edges.hashCode());
		return result;
	}

	//Modified this to not use IDs
	@Override
	public boolean equals(Object obj) {
		boolean result = false;
		if (obj instanceof Vertex){
			Vertex other = (Vertex) obj;
			//System.out.println("Comparing :"+this+" and "+other);
			if (c.equals(other.getC())) {
				//System.out.println("They are the same ! ");
				result = true;
			} 
		}
		return result;
	}

	

	//public void setId(int id) {
	//	this.id = id;
	//}



	public ASVConfig getC() {
		return c;
	}


	public void setC(ASVConfig c) {
		this.c = c;
	}
	
	public String toString() {
		return "Vertex : "+c.toString();//getId();
	}
	
	public void setParent(Vertex x) {
		this.parent = x;
	}
	
	public Vertex getParent() {
		return parent;
	}
	
	
	

	
}

