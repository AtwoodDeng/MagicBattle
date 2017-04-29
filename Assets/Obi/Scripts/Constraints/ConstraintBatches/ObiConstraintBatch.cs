using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;

namespace Obi
{
	/**
	 * ObiConstraintBatch are groups of constraints of the same type that are added/removed together from the solver.
	 * Batches can be of two main types:
	 *
	 * - Cooked: cooked batches reorder their constraints for better data locality and to enable the vectorized code path.
	 *			 Once the cooking proccess has finished constraints can't be added or removed, only activated/deactivated,
	 *			 but they yield very good performance.
	 *
	 * - Raw: raw batches allow for efficient addition/removal of constraints on the fly, and very efficient activation/deactivation.
	 *		  They do not require cooking but perform worse than cooked batches, as they don't do anyhting special regarding
	 *	      vectorization or improving cache data locality.
	 *
	 * ObiBatchedConstraints hold parameters for a group of batches of the same type.
	 */
	[Serializable]
	public abstract class ObiConstraintBatch
	{
		protected IntPtr batch;	/**< pointer to constraint batch in the solver.*/

		public const float MAX_YOUNG_MODULUS = 0.02f; //that of silicone elastomer (N/m2);
		public const float MIN_YOUNG_MODULUS = 0.0001f; //that of polymer foam (N/m2);

		[SerializeField][HideInInspector] protected int constraintCount = 0; /**< amount of constraints in this batch*/
		[SerializeField][HideInInspector] protected bool cooked = false;	 /**< returns whether this batch supports cooking or not.*/
		[SerializeField][HideInInspector] protected bool sharesParticles = false; /**< returns whether this batch must be done serially or can benefit from multithreading.*/

		[SerializeField][HideInInspector] protected List<int> activeConstraints = new List<int>();		/**< list of active constraint indices.*/
		[SerializeField][HideInInspector] protected List<int> phaseSizes = new List<int>();		/**< phase sizes for cooked batches.*/

		public IntPtr OniBatch{
			get{return batch;}
		}

		public int ConstraintCount{
			get{return constraintCount;}
		}
		public bool IsCooked{
			get{return cooked;}
		}
		public bool SharesParticles{
			get{return sharesParticles;}
		}
		public IEnumerable<int> ActiveConstraints
		{
    		get{return activeConstraints.AsReadOnly();}
		}

		// Implement this method to provide info about the batch type:
		public abstract Oni.ConstraintType GetConstraintType();

		// Implement these method to provide a custom clearing/cooking/swapping implementation.
		public abstract void Clear();
		public virtual void Cook(){}

		// Implement these methods to perform custom actions when adding/removing the batch from the solver.
		protected abstract void OnAddToSolver(ObiBatchedConstraints constraints);
		protected abstract void OnRemoveFromSolver(ObiBatchedConstraints constraints);

		// Implement these methods to push/pull constraint data to/from the solver.
		public abstract void PushDataToSolver(ObiBatchedConstraints constraints);
		public abstract void PullDataFromSolver(ObiBatchedConstraints constraints);

		public abstract List<int> GetConstraintsInvolvingParticle(int particleIndex);

		protected float StiffnessToCompliance(float stiffness){
			return 1.0f/(stiffness * MAX_YOUNG_MODULUS + MIN_YOUNG_MODULUS);
		}

		public void ActivateConstraint(int index){
			if (!activeConstraints.Contains(index))
				activeConstraints.Add(index);
		}

		public void DeactivateConstraint(int index){
			activeConstraints.Remove(index);
		}
		
		public ObiConstraintBatch(bool cooked, bool sharesParticles){
			this.cooked = cooked;
			this.sharesParticles = sharesParticles;
		}

		public void AddToSolver(ObiBatchedConstraints constraints){

			// create a constraint batch:
			batch = Oni.CreateBatch((int)GetConstraintType(),cooked);
			Oni.AddBatch(constraints.Actor.Solver.OniSolver,batch,sharesParticles);

			// custom stuff:
			OnAddToSolver(constraints);
		}

		public void RemoveFromSolver(ObiBatchedConstraints constraints){

			// custom stuff:
			OnRemoveFromSolver(constraints);

			// remove the constraint batch from the solver 
			// (no need to destroy it as its destruction is managed by the solver)
			Oni.RemoveBatch(constraints.Actor.Solver.OniSolver,batch);

			// important: set the batch pointer to null, as it could be destroyed by the solver.
			batch = IntPtr.Zero;

		}

		public void SetActiveConstraints(){
			Oni.SetActiveConstraints(batch,activeConstraints.ToArray(),activeConstraints.Count);
		}

		public void Enable(){
			Oni.EnableBatch(batch,true);
		}
	
		public void Disable(){
			Oni.EnableBatch(batch,false);
		}

	}
}

