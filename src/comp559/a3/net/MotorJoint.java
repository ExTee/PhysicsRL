package comp559.a3;

import javax.vecmath.Vector2d;
import javax.vecmath.Matrix3d;

public class MotorJoint {
	
	public Vector2d linearTarget;
	
	public double angularTarget;
	
	public double correctionFactor;
	
	public double maximumForce;
	
	public double maximumTorque;
	
	// current state
	
	/** The pivot mass; K = J * Minv * Jtrans */
	private Matrix3d K;
		
	/** The mass for the angular constraint */
	private double angularMass;
		
	/** The calculated linear error in the target distance */
	private Vector2d linearError;
		
	/** The calculated angular error in the target angle */
	private double angularError;

	// output
		
	/** The impulse applied to reduce linear motion */
	private Vector2d linearImpulse;
		
	/** The impulse applied to reduce angular motion */
	private double angularImpulse;
	
	public MotorJoint(RigidBody body1, RigidBody body2) {

	}
}
