using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;

namespace Obi{

/**
 * Holds information about distance constraints for an actor.
 */
[Serializable]
public class ObiDistanceConstraintBatch : ObiConstraintBatch
{

	[HideInInspector] public List<int> springIndices = new List<int>();					/**< Distance constraint indices.*/
	[HideInInspector] public List<float> restLengths = new List<float>();				/**< Rest distances.*/
	[HideInInspector] public List<Vector2> stiffnesses = new List<Vector2>();			/**< Stiffnesses of distance constraits.*/

	int[] solverIndices = new int[0];

	public ObiDistanceConstraintBatch(bool cooked, bool sharesParticles) : base(cooked,sharesParticles){
	}

	public override Oni.ConstraintType GetConstraintType(){
		return Oni.ConstraintType.Distance;
	}

	public override void Clear(){
		activeConstraints.Clear();
		springIndices.Clear();
		restLengths.Clear();
		stiffnesses.Clear();
		constraintCount = 0;	
	}

	public void AddConstraint(int index1, int index2, float restLength, float stretchStiffness, float compressionStiffness){
		activeConstraints.Add(constraintCount);
		springIndices.Add(index1);
		springIndices.Add(index2);
		restLengths.Add(restLength);
        stiffnesses.Add(new Vector2(stretchStiffness,compressionStiffness));
		constraintCount++;
	}

	public void RemoveConstraint(int index){
		activeConstraints.Remove(index);
		for(int i = 0; i < activeConstraints.Count; ++i)
		    if (activeConstraints[i] > index) activeConstraints[i]--;

		springIndices.RemoveRange(index*2,2);
		restLengths.RemoveAt(index);
        stiffnesses.RemoveAt(index);
		constraintCount--;
	}
	
	public override List<int> GetConstraintsInvolvingParticle(int particleIndex){
	
		List<int> constraints = new List<int>(10);
		
		for (int i = 0; i < ConstraintCount; i++){
			if (springIndices[i*2] == particleIndex || springIndices[i*2+1] == particleIndex) 
				constraints.Add(i);
		}
		
		return constraints;
	}

	public override void Cook()
	{
		batch = Oni.CreateBatch((int)Oni.ConstraintType.Distance,true);

		// Send initial data to batch:
		Oni.SetDistanceConstraints(batch,springIndices.ToArray(),restLengths.ToArray(),stiffnesses.ToArray(),ConstraintCount);

		// cook the batch and retrieve new sorted data:
		if (Oni.CookBatch(batch))
		{
			constraintCount = Oni.GetBatchConstraintCount(batch);
			activeConstraints = Enumerable.Range(0, constraintCount).ToList();

			int[] cookedIndices = new int[constraintCount*2]; 
			float[] cookedRestLengths = new float[constraintCount]; 
			Vector2[] cookedStiffnesses = new Vector2[constraintCount]; 

			Oni.GetDistanceConstraints(batch,cookedIndices,cookedRestLengths,cookedStiffnesses);

			springIndices = new List<int>(cookedIndices);
			restLengths = new List<float>(cookedRestLengths);
			stiffnesses = new List<Vector2>(cookedStiffnesses);

			int phaseCount = Oni.GetBatchPhaseCount(batch);
			int[] phases = new int[phaseCount];
			Oni.GetBatchPhaseSizes(batch,phases);
			this.phaseSizes = new List<int>(phases);
		}

		Oni.DestroyBatch(batch);
		batch = IntPtr.Zero;
	}

	protected override void OnAddToSolver(ObiBatchedConstraints constraints){

		// Set solver constraint data:
		solverIndices = new int[springIndices.Count];
		for (int i = 0; i < restLengths.Count; i++)
		{
			solverIndices[i*2] = constraints.Actor.particleIndices[springIndices[i*2]];
			solverIndices[i*2+1] = constraints.Actor.particleIndices[springIndices[i*2+1]];
		}

	}

	protected override void OnRemoveFromSolver(ObiBatchedConstraints constraints){
	}

	public override void PushDataToSolver(ObiBatchedConstraints constraints){ 

		if (constraints == null || constraints.Actor == null || !constraints.Actor.InSolver)
			return;

		ObiDistanceConstraints dc = (ObiDistanceConstraints) constraints;

		float[] scaledRestLengths = new float[restLengths.Count];
		
		for (int i = 0; i < restLengths.Count; i++){
			scaledRestLengths[i] = restLengths[i]*dc.stretchingScale;
			stiffnesses[i] = new Vector2(StiffnessToCompliance(dc.stiffness),dc.slack*scaledRestLengths[i]);
		}

		Oni.SetDistanceConstraints(batch,solverIndices,scaledRestLengths,stiffnesses.ToArray(),ConstraintCount);
		Oni.SetBatchPhaseSizes(batch,phaseSizes.ToArray(),phaseSizes.Count);
	}

	public override void PullDataFromSolver(ObiBatchedConstraints constraints){
	}	

}
}
