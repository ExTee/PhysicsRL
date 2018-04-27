package comp559.a3;

import javax.vecmath.Vector2d;

public class Constraint {
	public RigidBody A;
	public RigidBody B;
	
	public Constraint( RigidBody A, RigidBody B) {
		this.A = A;
		this.B = B;
		
		if(!(A.minv > 0 || B.minv > 0)){
			System.out.println("Constraint between two infinite mass bodies not allowed");
		}
	}
	
	public void ApplyImpulse(Vector2d I){
		Vector2d I_A = new Vector2d(I);
		I_A.scale(A.minv);
		//I_A.scale(-1);
		A.v.add(I_A);
		
		Vector2d I_B = new Vector2d(I);
		I_B.scale(B.minv);
		//I_B.scale(-1);
		B.v.add(I_B);
	}
	
	public void Solve(double dt){
		
	}
}
