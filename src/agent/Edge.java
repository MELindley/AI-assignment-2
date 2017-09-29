package agent;

import problem.ASVConfig;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Collections;

public class Edge {
	Vertex v1;
	Vertex v2;
	double weight;
	ArrayList<ASVConfig> primitiveSteps;

	public Edge() {
		v1 = new Vertex();
		v2 = new Vertex();
		weight = 0;
		primitiveSteps = new ArrayList<ASVConfig>();
	}

	public Edge(Vertex v1, Vertex v2) {
		this.v1 = v1;
		this.v2 = v2;
		this.weight = calculateWeight();
		primitiveSteps = new ArrayList<ASVConfig>();
	}
	
	public boolean contains(Vertex v){
		return (v==v1)||(v==v2);
	}
	
	public Vertex getOther(Vertex v){
		if(v == v1){
			return v2;
		}
		if(v == v2){
			return v1;
		}
		return new Vertex();
	}
	
	public Vertex getV1() {
		return this.v1;
	}

	public Vertex getV2() {
		return this.v2;
	}
	
	/*Calculates the weight of an edge
	 * The Weight of an edge is defined as the total distance between the 2 ASVConfigs in its connected vertices
	 * 
	*/
	private double calculateWeight() {
		
		ASVConfig c1 = this.v1.getC();
		ASVConfig c2 = this.v2.getC();
		
		//Define weight as the total distance between the 2 configuration
		return c1.totalDistance(c2);
		
	}
	
	
	public double getWeight() {
		return weight;
	}

	@Override
	public boolean equals(Object obj){
		boolean result = false;
		if(obj instanceof Edge){
			Edge e = (Edge) obj;
			result = (e.getV1()==this.getV1() && e.getV2() == this.getV2())
					||(e.getV1()==this.getV2() && e.getV2() == this.getV1());
		}
		return result;
	}
	
	@Override
	public int hashCode() {
		final int prime = 7;
		int result = 1;
		result = prime * result + (v1.hashCode()+v2.hashCode());
		return result;
	}
	
	@Override
	public String toString(){
		return v1.getC()+" - "+v2.getC();
	}

	/**
	 * Returns the primitive steps of this edge.
	 * Take in the vertex to start with
	 * @param from Vertex we are starting from
	 * @return
	 */
	public ArrayList<ASVConfig> getPrimitiveSteps(Vertex from) {
		if(primitiveSteps.get(0).equals(from))
			return primitiveSteps;
		else
			//reverse the primitive steps and return them
			Collections.reverse(primitiveSteps);
			return primitiveSteps;
	}

	public void setPrimitiveSteps(ArrayList<ASVConfig> primitiveSteps) {
		this.primitiveSteps = primitiveSteps;
	}

	
}
