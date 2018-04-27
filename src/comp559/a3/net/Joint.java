package comp559.a3;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

public class Joint {
	
	public RigidBody body1;
	public RigidBody body2;
	
	public Vector2d localAnchor1;
	public Vector2d localAnchor2;
	
	/** Distance between two anchor points*/
	public double distance;
	public double frequency = 0.1;
	public double dampingRatio = 0.98;
	
	/** Normal */
	public Vector2d n;
	
	/** The effective mass of the two body system (Kinv = J * Minv * Jtrans) */
	private double invK;
	
	/** Accumulated impulse */
	public double impulse;
	
	/** Bias for adding work to constraint (spring simulation) */
	public double bias;
	
	/** Damping of constraint */
	public double gamma;
	
	public Joint(RigidBody body1, RigidBody body2, Vector2d anchor1, Vector2d anchor2) {
		this.body1 = body1;
		this.body2 = body2;
		
		//this.localAnchor1 = anchor1;
		//this.localAnchor1 = anchor2;
		this.localAnchor1 = body1.getLocalPoint(anchor1);
		this.localAnchor2 = body2.getLocalPoint(anchor2);
		
		//Set initial distance
		Vector2d d = new Vector2d(0,0);
		d.sub(anchor1, anchor2);
		this.distance = d.length();
	}
	
	public String toString() {
		StringBuilder sb = new StringBuilder();
		sb.append("DistanceJoint[").append(super.toString())
		  .append("|Anchor1=").append(this.getAnchor1())
		  .append("|Anchor2=").append(this.getAnchor2())
		  .append("|Frequency=").append(this.frequency)
		  .append("|DampingRatio=").append(this.dampingRatio)
		  .append("|Distance=").append(this.distance)
		  .append("]");
		return sb.toString();
	}

	public void display(GLAutoDrawable drawable) {
	      final GL2 gl = drawable.getGL().getGL2();
	      gl.glLineWidth( 5 );
	      //drawing the base
	      gl.glBegin (GL2.GL_LINES);
	      gl.glColor3f(0.000f, 0.000f, 1.00f);
	      //gl.glVertex2d(this.getAnchor1().x, this.getAnchor1().y);
	      //gl.glVertex2d(this.getAnchor2().x, this.getAnchor2().y);
	      gl.glVertex2d(this.body1.x.x, this.body1.x.y);
	      gl.glVertex2d(this.body2.x.x, this.body2.x.y);
	      gl.glEnd();
	   
	   }

	/**
	 * Advances the joint one step in time
	 * @param dt: step size
	 */
	public void advanceTime(double dt) {
		// TODO Auto-generated method stub
		initializeConstraints(dt);
		solveVelocityConstraints(dt);
		solvePositionConstraints(dt);
		
		
		
		//body1.updateTransformations();
		//body2.updateTransformations();
	}
	
	public void initializeConstraints(double dt) {
		
		this.localAnchor1 = body1.getLocalPoint(new Vector2d(body1.x));
		this.localAnchor2 = body2.getLocalPoint(new Vector2d(body2.x));
		//Vector2d tmp = new Vector2d(0,0);
		//tmp.sub(body1.x, body2.x);
		//this.distance = tmp.length();
		
		/** Default linear tolerance*/
		double linearTolerance = 0.005;
		
		//Maybe W2B
		RigidTransform t1 = body1.transformB2W;
		RigidTransform t2 = body2.transformB2W;
		
		//double m1 = body1.massLinear;
		//double m2 = body2.massLinear;
		
		double invM1 = body1.minv;
		double invM2 = body2.minv;
		double invI1 = body1.jinv;
		double invI2 = body2.jinv;
		
		//Compute normal
		Vector2d r1 = new Vector2d(0,0);
		//Vector2d b12la1 = new Vector2d(this.localAnchor1.x - this.body1.x.x, this.localAnchor1.y - this.body1.x.y);	//b1 CoM to anchor1
		Vector2d b12la1 = new Vector2d(this.localAnchor1.x, this.localAnchor1.y);
		t1.transform(b12la1, r1);
		
		//Create a copy of r1
		Vector2d r1_p = new Vector2d(r1.x,r1.y);
		
		Vector2d r2 = new Vector2d(0,0);
		//Vector2d b12la2 = new Vector2d(this.localAnchor2.x - this.body2.x.x, this.localAnchor2.y - this.body2.x.y);	//b2 CoM to anchor2
		Vector2d b12la2 = new Vector2d(this.localAnchor2.x, this.localAnchor2.y);
		t2.transform(b12la2, r2);
		
		//Create a copy of r2
		Vector2d r2_p = new Vector2d(r2.x,r2.y);
		
		Vector2d b1_center = new Vector2d(body1.x.x,body1.x.y);	
		Vector2d b2_center = new Vector2d(body2.x.x,body2.x.y);
		
		r1_p.add(b1_center);
		r2_p.add(b2_center);		
		r1_p.sub(r2_p);
		
		this.n = r1_p;
		
		//Check tolerance
		double length = n.length();
		if (length < linearTolerance) {
			this.n.set(0, 0);
		} else{
			//normalize
			this.n.scale(1.0 / length);
		}
		
		//Compute K inverse
		
		//Cross product r1 and n
		double cr1n = cross(r1, this.n);
		double cr2n = cross(r2, this.n);
		double invMass = invM1 + invI1 * cr1n * cr1n;
		invMass += invM2 + invI2 * cr2n * cr2n;

		//Maybe check for zero first?
		this.invK = 1.0 / invMass;
		
		/**TODO : Spring Damping here */
		
		if (this.frequency > 0.0) {
			double x = length - this.distance;
			double w = Math.PI * this.frequency;
			double d  = 2.0 * this.invK * this.dampingRatio * w;
			double k = this.invK * w * w;
			
			this.gamma = dt * (d + dt * k);
			this.gamma = 1.0/this.gamma;
			this.bias = x * dt * k + this.gamma;
			invMass += this.gamma;
			this.invK = 1.0 / invMass;
		} else{
			this.bias = 0.0;
			this.gamma = 0.0;
		}
		

		
		//warm start
		impulse = impulse * dt;
		
		//J = impulse * n
		Vector2d J = new Vector2d(this.n.x * impulse , this.n.y * impulse);
		
		//Update Body1 velocities
		Vector2d J_b1 = new Vector2d(J);
		J_b1.scale(invM1);
		body1.v.add(J_b1);
		body1.omega += invI1 * cross(r1,J);
		
		//Update Body2 velocities
		Vector2d J_b2 = new Vector2d(J);
		J_b2.scale(invM2);
		body2.v.sub(J_b2);	//this is a subtraction because we scaled the above by negative value
		body2.omega -= invI2 * cross(r2,J);
	}
	
	public void solveVelocityConstraints(double dt){
		RigidTransform t1 = body1.transformB2W;
		RigidTransform t2 = body2.transformB2W;
		
		double invM1 = body1.minv;
		double invM2 = body2.minv;
		double invI1 = body1.jinv;
		double invI2 = body2.jinv;		
		
		//compute r1 and r2
		Vector2d r1 = new Vector2d(0,0);
		//Vector2d b12la1 = new Vector2d(this.localAnchor1.x - this.body1.x.x, this.localAnchor1.y - this.body1.x.y);	//b1 CoM to anchor1
		Vector2d b12la1 = new Vector2d(this.localAnchor1.x, this.localAnchor1.y);
		t1.transform(b12la1, r1);
		
		Vector2d r2 = new Vector2d(0,0);
		//Vector2d b12la2 = new Vector2d(this.localAnchor2.x - this.body2.x.x, this.localAnchor2.y - this.body2.x.y);	//b2 CoM to anchor2
		Vector2d b12la2 = new Vector2d(this.localAnchor2.x, this.localAnchor2.y);
		t2.transform(b12la2, r2);
		
		//Compute relative v
		Vector2d v1 = new Vector2d(body1.v);
		v1.add(cross(r1, body1.omega));
		
		Vector2d v2 = new Vector2d(body2.v);
		v2.add(cross(r2,body2.omega));
		
		//compute Jv
		Vector2d temp = new Vector2d();
		temp.sub(v1, v2);
		double Jv = n.dot(temp);
		
		//Compute lambda (magnitude of impulse)
		double j = -this.invK * (Jv + this.bias + this.gamma + this.impulse);
		this.impulse += j;
		
		
		Vector2d J = new Vector2d();
		J.add(n);
		J.scale(j);
		
		//apply impulse to body 1
		Vector2d J_b1 = new Vector2d(J);
		J_b1.scale(invM1);
		body1.v.add(J_b1);
		body1.omega += invI1 * cross(r1,J);
		
		
		//apply impulse to body 2
		
		Vector2d J_b2 = new Vector2d(J);
		J_b2.scale(invM2);
		body2.v.sub(J_b2);
		body2.omega -= invI2 * cross(r2,J);			
	}
	
	public boolean solvePositionConstraints(double dt) {
		//If this is a spring damper, don't solve position constraints
		if (this.frequency > 0.0) {
			return true;
		}
		
		/** Default values for lT and mLC */
		double linearTolerance = 0.005;
		double maxLinearCorrection = 0.2;
		
		RigidTransform t1 = body1.transformB2W;
		RigidTransform t2 = body2.transformB2W;
		
		double invM1 = body1.minv;
		double invM2 = body2.minv;
		double invI1 = body1.jinv;
		double invI2 = body2.jinv;
		
		//Recompute normal in-case it changed during integration
		Vector2d r1 = new Vector2d(0,0);
		//Vector2d b12la1 = new Vector2d(this.localAnchor1.x - this.body1.x.x, this.localAnchor1.y - this.body1.x.y);	//b1 CoM to anchor1
		Vector2d b12la1 = new Vector2d(this.localAnchor1.x, this.localAnchor1.y);
		t1.transform(b12la1, r1);
		
		//Create a copy of r1
		Vector2d r1_p = new Vector2d(r1.x,r1.y);
		
		Vector2d r2 = new Vector2d(0,0);
		//Vector2d b12la2 = new Vector2d(this.localAnchor2.x - this.body2.x.x, this.localAnchor2.y - this.body2.x.y);	//b2 CoM to anchor2
		Vector2d b12la2 = new Vector2d(this.localAnchor2.x, this.localAnchor2.y);
		t2.transform(b12la2, r2);
		
		//Create a copy of r2
		Vector2d r2_p = new Vector2d(r2.x,r2.y);
		
		Vector2d b1_center = new Vector2d(body1.x.x,body1.x.y);	
		Vector2d b2_center = new Vector2d(body2.x.x,body2.x.y);
		
		r1_p.add(b1_center);
		r2_p.add(b2_center);		
		r1_p.sub(r2_p);
		
		this.n = r1_p;
		
		//Solve position constraint
		n.normalize();
		double l = n.length();
		double C = l - this.distance;
		C = clamp(C, -maxLinearCorrection, maxLinearCorrection);
		
		double impulse = -this.invK * C;
		
		Vector2d J = new Vector2d(this.n);
		J.scale(impulse);
		
		Vector2d b1_translation = new Vector2d(J);
		b1_translation.scale(invM1);
		//body1.translate(b1_translation)
		double b1_rotation_theta = invI1 * cross(r1, J);	
		//body1.rotate(b1_rotation_theta, c1)
		body1.theta += b1_rotation_theta;
		body1.x = new Point2d(b1_translation);
		
		//body1.transformB2W.set(b1_rotation_theta, b1_translation);
		//body1.transformB2W.transform(body1.x);
		
		Vector2d b2_translation = new Vector2d(J);
		b2_translation.scale(invM2);
		b2_translation.scale(-1);
		//body2.translate(b2_translation)
		double b2_rotation_theta = -invI2 * cross(r2, J);
		//body2.rotate(b2_rotation_theta, c2)	
		//body2.transformB2W.set(b2_rotation_theta, b2_translation);
		//body2.transformB2W.transform(body2.x);
		body2.theta += b2_rotation_theta;
		body2.x = new Point2d(b2_translation);
		//body2.
		
		return Math.abs(C) < linearTolerance;
		
		
	}
	
	/**
	 * Computes the Cross product between vectors A and B
	 * @param A
	 * @param B
	 * @return A X B
	 */
	public double cross(Vector2d A, Vector2d B){
		return A.x * B.y - A.y * B.x;
	}
	
	/**
	 * Returns cross product of A and z value of right
	 * @param A
	 * @param z
	 * @return
	 */
	public Vector2d cross(Vector2d A, double z){
		return new Vector2d(-1.0 * A.y * z, A.x * z);
	}

	public double clamp(double value, double min, double max)
	{
		if(value < min){
			return min;
		}
		if(value > max){
			return max;
		}
		
		return value;
	}
	
	public Vector2d getAnchor1(){
		return body1.getWorldPoint(this.localAnchor1);
	}
	
	public Vector2d getAnchor2(){
		return body2.getWorldPoint(this.localAnchor2);
	}
}
