package comp559.a3;

import javax.vecmath.Vector2d;

public class DistanceConstraint extends Constraint {
	private double distance;

	public DistanceConstraint(RigidBody A, RigidBody B, double distance) {
		super(A, B);
		this.distance = distance;
	}

	public void Solve(double dt){
		Vector2d axis = new Vector2d(this.A.x.x - this.B.x.x, this.A.x.y - this.B.x.y);
		double cur_distance = axis.length();
		Vector2d unit_axis = new Vector2d();
		unit_axis.normalize(axis);
		
		// calculate relative velocity in the axis, we want to remove this
		
		Vector2d temp = new Vector2d(this.B.v);
		temp.sub(this.A.v);
		double relVel = temp.dot(unit_axis);
		
		double relDist = cur_distance - this.distance;
		
		// calculate impulse to solve
		
		double remove = relVel + relDist / dt;
		double impulse = remove / (this.A.minv + this.B.minv);
		
		Vector2d I = new Vector2d(unit_axis);
		I.scale(impulse);
		
		ApplyImpulse(I);
		
	}
	
}
