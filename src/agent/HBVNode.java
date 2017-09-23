package agent;

import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;
import java.awt.Shape;
import java.util.ArrayList;
import java.util.List;

public class HBVNode {
    private List<HBVNode> children = new ArrayList<HBVNode>();
    //private HBVNode parent = null;
    private Rectangle2D volume = null;
    private Object primitive= null;

    public HBVNode(){
        //Empty tree in the case of no Obstacles
    }

    public HBVNode(Line2D.Double primitive) {
	        /*
	         * Constructor for a line primitive
	         */
        this.primitive = primitive;
        this.volume = primitive.getBounds2D();
   
    }
    
    public HBVNode(HBVNode node1, HBVNode node2){
    	/*
    	 * Constructor for HBVNode Emcompassing other 2 nodes
    	 */
    	Rectangle2D volume1 = node1.getVolume();
    	Rectangle2D volume2 = node2.getVolume();
    	this.volume = new Rectangle2D.Double(Math.min(volume1.getX(),volume2.getX()),
    										 Math.min(volume1.getY(), volume2.getY()),
    										 Math.max(volume1.getX()+volume1.getWidth(),volume2.getX()+volume2.getWidth()),
    										 Math.max(volume1.getY()+volume1.getHeight(), volume2.getY()+volume2.getHeight())
    										 );
    	this.children.add(node1);
    	this.children.add(node2);
    }

//    public HBVNode(Object primitive, HBVNode parent) {
//        this.primitive = primitive;
//        if(primitive instanceof Line2D){
//            Line2D.Double l= (Line2D.Double)primitive;
//            this.volume = l.getBounds2D();
//        }else{
//            if(primitive instanceof Ellipse2D){
//                this.volume =(Rectangle2D.loat)primitive;
//            }
//        }
//        this.parent = parent;
//    }

    public List<HBVNode> getChildren() {
        return children;
    }

//    public void setParent(HBVNode parent) {
//        // parent.addChild(this);
//        this.parent = parent;
//    }

    public void addChild(HBVNode child) {
        this.children.add(child);
    }

    public Rectangle2D getVolume() {
        return this.volume;
    }
	    
	    
	    /*public void setVolume(T data) {
	        this.data = data;
	    }*/

    public Object getPrimitive() {
        return primitive;
    }

    public void setPrimitive(Object primitive) {
        this.primitive = primitive;
    }

//    public boolean isRoot() {
//        return (this.parent == null);
//    }

    public boolean isLeaf() {
        if(this.children.size() == 0)
            return true;
        else
            return false;
    }

//    public void removeParent() {
//        this.parent = null;
//    }

//    public boolean isEmpty(){
//        if((children.size() ==0)&& parent == null && volume == null && primitive == null)
//            return true;
//        return false;
//    }

    @Override
    public String toString(){
        return volume+" Children: "+ children+" primitive: "+ primitive;
    }
    
}