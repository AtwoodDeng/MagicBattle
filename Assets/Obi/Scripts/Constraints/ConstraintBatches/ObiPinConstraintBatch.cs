using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;

namespace Obi{

/**
 * Holds information about distance constraints for an actor.
 */
[Serializable]
public class ObiPinConstraintBatch : ObiConstraintBatch
{

	[HideInInspector] public List<int> pinIndices = new List<int>();					/**< Pin constraint indices.*/
	[HideInInspector] public List<Collider> pinBodies = new List<Collider>();			/**< Pin bodies.*/
	[HideInInspector] public List<Vector4> pinOffsets = new List<Vector4>();			/**< Offset expressed in the attachment's local space.*/
	[HideInInspector] public List<float> stiffnesses = new List<float>();				/**< Stiffnesses of pin constraits.*/

	[HideInInspector] public List<float> pinBreakResistance = new List<float>(); 	/**< Per-constraint tear resistances.*/

	int[] solverIndices = new int[0];

	public ObiPinConstraintBatch(bool cooked, bool sharesParticles) : base(cooked,sharesParticles){
	}

	public override Oni.ConstraintType GetConstraintType(){
		return Oni.ConstraintType.Pin;
	}

	public override void Clear(){
		activeConstraints.Clear();
		pinIndices.Clear();
		pinBodies.Clear();
		pinOffsets.Clear();
		stiffnesses.Clear();
		pinBreakResistance.Clear();
		constraintCount = 0;	
	}

	public void AddConstraint(int index1, Collider body, Vector3 offset, float stiffness){
		activeConstraints.Add(constraintCount);
		pinIndices.Add(index1);
		pinBodies.Add(body);
		pinOffsets.Add(offset);
        stiffnesses.Add(stiffness);
		pinBreakResistance.Add(float.MaxValue);
		constraintCount++;
	}

	public void RemoveConstraint(int index){
		activeConstraints.Remove(index);

		for(int i = 0; i < activeConstraints.Count; ++i)
		    if (activeConstraints[i] > index) activeConstraints[i]--;

		pinIndices.RemoveAt(index);
		pinBodies.RemoveAt(index);
        pinOffsets.RemoveAt(index);
		stiffnesses.RemoveAt(index);
		pinBreakResistance.RemoveAt(index);
		constraintCount--;
	}
	
	public override List<int> GetConstraintsInvolvingParticle(int particleIndex){
	
		List<int> constraints = new List<int>(5);
		
		for (int i = 0; i < ConstraintCount; i++){
			if (pinIndices[i] == particleIndex)
				constraints.Add(i);
		}
		
		return constraints;
	}

	protected override void OnAddToSolver(ObiBatchedConstraints constraints){

		// Set solver constraint data:
		solverIndices = new int[pinIndices.Count*2];
		for (int i = 0; i < pinOffsets.Count; i++)
		{
			solverIndices[i*2] = constraints.Actor.particleIndices[pinIndices[i]];
		}

		UpdateColliderIndices(constraints.Actor.Solver.colliderGroup);

	}

	public void UpdateColliderIndices(ObiColliderGroup group){

		// Set solver constraint data:
		for (int i = 0; i < pinOffsets.Count; i++)
		{
			// each time the collider list changes, update pin constraint indices.
			// each time the collider group changes, update pin constraint indices, adding the collider to the new group if needed.

			// if a collider group exists, get collider indices:
			if (group != null) {

				int colliderIndex = group.GetIndexOfCollider(pinBodies[i]);

				// if the collider does not exists in the collider group, insert it at the end.
				if (colliderIndex == -1){
					colliderIndex = group.ColliderCount;
					group.AddCollider(pinBodies[i]);
				}

				solverIndices[i*2+1] = colliderIndex;
				
			}else{
				solverIndices[i*2+1] = -1;
			}
		}
	}

	protected override void OnRemoveFromSolver(ObiBatchedConstraints constraints){
	}

	public override void PushDataToSolver(ObiBatchedConstraints constraints){ 

		if (constraints == null || constraints.Actor == null || !constraints.Actor.InSolver)
			return;

		ObiPinConstraints pc = (ObiPinConstraints) constraints;

		for (int i = 0; i < stiffnesses.Count; i++){
			stiffnesses[i] = StiffnessToCompliance(pc.stiffness);
		}

		Oni.SetPinConstraints(batch,solverIndices,pinOffsets.ToArray(),stiffnesses.ToArray(),ConstraintCount);

	}

	public override void PullDataFromSolver(ObiBatchedConstraints constraints){
	}	

	public void BreakConstraints(){

		float[] forces = new float[ConstraintCount];
		Oni.GetBatchConstraintForces(batch,forces,ConstraintCount,0);

		for (int i = 0; i < forces.Length; i++){
			if (-forces[i] * 1000 > pinBreakResistance[i]){ // units are kilonewtons.
				activeConstraints.Remove(i);
			}
		}

		SetActiveConstraints();
	}

}
}
