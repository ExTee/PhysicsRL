package comp559.a3;

import javax.vecmath.Vector2d;

import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

public class Magnet {

	public RigidBody body1;
	public RigidBody body2;
	
	public Magnet(RigidBody body1 , RigidBody body2) {
		this.body1 = body1;
		this.body2 = body2;
	}
	
    /**
     * All springs share the same stiffness coefficient
     */
    public static double k = 9999;
    
    /**
     * All springs share the same damping coefficient
     */
    public static double b = 0.7;
    
    /**
     * Rest length
     */
    public double l0 = 0;
    

    /**
     * Computes the rest length of the connected particles
     */
    public void computeRestLength() {
    	Vector2d tmp = new Vector2d();
    	tmp.sub(this.body1.x0, this.body2.x0);
        l0 = tmp.length();
    }
    
    /**
     * Sets the rest length of the connected particles with their current positions
     */
    public void setRestLength(double l) {
        l0 = l;
    }
    
    /**
     * Applies spring forces to the two particles
     */
    public boolean apply() {
        Vector2d force = new Vector2d();
        
        Vector2d p1 = new Vector2d(this.body1.x);
        Vector2d p2 = new Vector2d(this.body2.x);
        
        force.sub(p2,p1);
        
        if(force.length() > this.l0 * 0.6){
        	return true;
        	//Don't do anything if distance > rest distance
        }
        else
        {
            double l = force.length();
            force.normalize();
            force.scale( (l-l0)*k );
            this.body1.force.add(force);
            force.scale(-1);
            this.body2.force.add(force);
            
            force.sub( p2, p1 );
            force.normalize();
            Vector2d v = new Vector2d();
            v.sub(this.body2.v, this.body1.v);
            double rv = force.dot(v);
            force.scale( b * rv );
            this.body1.force.add(force);
            force.scale(-1);
            this.body2.force.add(force); 
        }
        return true;

           
    }
}
