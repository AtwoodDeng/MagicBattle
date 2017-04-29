using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;

namespace Obi
{

/**
 * An ObiCloth component generates a particle-based physical representation of the object geometry
 * to be feeded to an ObiSolver component. To do that, it needs connectivity information about the mesh,
 * which is provided by an ObiMeshConnectivity asset.
 * 
 * You can use it to make flags, capes, jackets, pants, ropes, drapes, nets, or any kind of object that exhibits cloth-like behavior.
 * 
 * ObiCloth objects have their particle properties expressed in local space. That means that particle positions, velocities, etc
 * are all expressed and serialized using the object's transform as reference. Thanks to this it is very easy to instantiate cloth prefabs and move/rotate/scale
 * them around, while keeping things working as expected. 
 * 
 * For convenience, solver gravity is expressed and applied in world space. 
 * Which means that no matter how you rotate a ObiCloth object, gravity will always pull particles down.
 * (as long as gravity in your solver is meant to pulls things down, heh).
 * 
 */
[ExecuteInEditMode]
[AddComponentMenu("Physics/Obi/Obi Cloth")]
[RequireComponent(typeof (ObiSkinConstraints))]
[RequireComponent(typeof (ObiVolumeConstraints))]
[RequireComponent(typeof (ObiTetherConstraints))]
public class ObiCloth : ObiClothBase
{

	protected SkinnedMeshRenderer skinnedMeshRenderer;
	protected Animator animator;

	public ObiSkinConstraints SkinConstraints{
		get{return constraints[Oni.ConstraintType.Skin] as ObiSkinConstraints;}
	}
	public ObiVolumeConstraints VolumeConstraints{
		get{return constraints[Oni.ConstraintType.Volume] as ObiVolumeConstraints;}
	}
	public ObiTetherConstraints TetherConstraints{
		get{return constraints[Oni.ConstraintType.Tether] as ObiTetherConstraints;}
	}

	public bool IsSkinned{
		get{return skinnedMeshRenderer != null;}
	}
	
	public override Matrix4x4 ActorLocalToWorldMatrix{
		get{
			if (!Application.isPlaying || skinnedMeshRenderer == null || skinnedMeshRenderer.rootBone == null) 
				return transform.localToWorldMatrix;
			return skinnedMeshRenderer.rootBone.localToWorldMatrix;
		}
	}

	public override Matrix4x4 ActorWorldToLocalMatrix{
		get{
			if (!Application.isPlaying || skinnedMeshRenderer == null || skinnedMeshRenderer.rootBone == null) 
				return transform.worldToLocalMatrix;
			return skinnedMeshRenderer.rootBone.worldToLocalMatrix;
		}
	}

	public override void Awake(){
		base.Awake();
		skinnedMeshRenderer = GetComponent<SkinnedMeshRenderer>();
		animator = GetComponentInParent<Animator>();
	}

	public override void OnEnable(){

		base.OnEnable();

		// Initialize cloth:
		if (skinnedMeshRenderer == null)
			InitializeWithRegularMesh();
		else 
			InitializeWithSkinnedMesh();

	}
		
	public override void OnDisable(){
		
		base.OnDisable();

		if (skinnedMeshRenderer != null)
			skinnedMeshRenderer.sharedMesh = sharedMesh;

	}
	
	public override bool AddToSolver(object info){

		if (Initialized && base.AddToSolver(info)){
				
			particleIndicesHandle = Oni.PinMemory(particleIndices);

			for (int i = 0; i < 16; ++i)
				transformData[i] = transform.worldToLocalMatrix[i];

			IntPtr skinbatch = IntPtr.Zero;
			if (SkinConstraints.GetBatches().Count > 0)
				skinbatch = SkinConstraints.GetBatches()[0].OniBatch;

			deformableMesh = Oni.CreateDeformableMesh(Solver.OniSolver,
													  topology.HalfEdgeMesh,
													  skinbatch,
													  transformData,
													  particleIndicesHandle.AddrOfPinnedObject(),
												      sharedMesh.vertexCount,
													  sharedMesh.vertexCount);

			Oni.SetDeformableMeshTBNUpdate(deformableMesh,normalsUpdate,updateTangents);

			GetMeshDataArrays(clothMesh);

			SetSkinnedMeshAnimationInfo();

			CallOnDeformableMeshSetup();

			return true;
		}
		return false;
    }

	public override bool RemoveFromSolver(object info){

		bool removed = false;

		try{

			if (solver != null && InSolver)
				Oni.DestroyDeformableMesh(Solver.OniSolver,deformableMesh);

			Oni.UnpinMemory(particleIndicesHandle);
			Oni.UnpinMemory(meshVerticesHandle);
			Oni.UnpinMemory(meshNormalsHandle);
			Oni.UnpinMemory(meshTangentsHandle);

			CallOnDeformableMeshTearDown();

		}catch(Exception e){
			Debug.LogException(e);
		}finally{
			removed = base.RemoveFromSolver(info);
		}
		return removed;
	}

	protected void SetSkinnedMeshAnimationInfo(){

		if (skinnedMeshRenderer != null){

			Matrix4x4[] rendererBindPoses = sharedMesh.bindposes;
			BoneWeight[] rendererWeights = sharedMesh.boneWeights;

			float[] bindPoses = new float[16*rendererBindPoses.Length];
			
			for (int p = 0; p < rendererBindPoses.Length; ++p){
				for (int i = 0; i < 16; ++i)
					bindPoses[p*16+i] = rendererBindPoses[p][i];
			}

			Oni.BoneWeights[] weights = new Oni.BoneWeights[rendererWeights.Length];
			for (int i = 0; i < rendererWeights.Length; ++i)
				weights[i] = new Oni.BoneWeights(rendererWeights[i]);

			Oni.SetDeformableMeshAnimationData(deformableMesh,bindPoses,weights,rendererBindPoses.Length);
		}
	}	

	protected void InitializeWithSkinnedMesh(){
	
        if (!Initialized)
           return;

        sharedMesh = sharedTopology.InputMesh;
		
		// Make a deep copy of the original shared mesh.
		clothMesh = Mesh.Instantiate(skinnedMeshRenderer.sharedMesh) as Mesh;
		
		// remove bone weights so that the mesh is not affected by skinning:
		clothMesh.boneWeights = new BoneWeight[]{};
		clothMesh.MarkDynamic();
		GetMeshDataArrays(clothMesh);
		
		if (Application.isPlaying)
			skinnedMeshRenderer.sharedMesh = clothMesh;

	}

	/**
	 * Generates the particle based physical representation of the cloth mesh. This is the initialization method for the cloth object
	 * and should not be called directly once the object has been created.
	 */
	public override IEnumerator GeneratePhysicRepresentationForMesh()
	{		
		initialized = false;
		initializing = false;
		
		if (sharedTopology == null){
			Debug.LogError("No ObiMeshConnectivity provided. Cannot initialize physical representation.");
			yield break;
		}else if (!sharedTopology.Initialized){
			Debug.LogError("The provided ObiMeshConnectivity contains no data. Cannot initialize physical representation.");
            yield break;
		}
		
		initializing = true;

		RemoveFromSolver(null);

		ResetTopology();

		active = new bool[topology.heVertices.Length];
		positions = new Vector3[topology.heVertices.Length];
		restPositions = new Vector4[topology.heVertices.Length];
		velocities = new Vector3[topology.heVertices.Length];
		vorticities = new Vector3[topology.heVertices.Length];
		invMasses  = new float[topology.heVertices.Length];
		solidRadii = new float[topology.heVertices.Length];
		phases = new int[topology.heVertices.Length];
		areaContribution = new float[topology.heVertices.Length]; 
		deformableTriangles = new int[topology.heFaces.Length*3]; 

		// Create a particle for each vertex:
		for (int i = 0; i < topology.heVertices.Length; i++){
			
			Oni.Vertex vertex = topology.heVertices[i];

			// Get the particle's area contribution.
			areaContribution[i] = 0;
			foreach (Oni.Face face in topology.GetNeighbourFacesEnumerator(vertex)){
				areaContribution[i] += topology.GetFaceArea(face)/3;
            }
			
			// Get the shortest neighbour edge, particle radius will be half of its length.
			float minEdgeLength = Single.MaxValue;
			foreach (Oni.HalfEdge edge in topology.GetNeighbourEdgesEnumerator(vertex)){
				minEdgeLength = Mathf.Min(minEdgeLength,Vector3.Distance(topology.heVertices[topology.GetHalfEdgeStartVertex(edge)].position,
					                                                     topology.heVertices[edge.endVertex].position));
			}

			active[i] = true;
			invMasses[i] = (skinnedMeshRenderer == null && areaContribution[i] > 0) ? (1.0f / (0.05f * areaContribution[i])) : 0;
			positions[i] = vertex.position;
			restPositions[i] = positions[i];
			restPositions[i][3] = 0; // activate rest position.
			solidRadii[i] = minEdgeLength * 0.5f;
			phases[i] = Oni.MakePhase(gameObject.layer,selfCollisions?Oni.ParticlePhase.SelfCollide:0);
			
			if (i % 500 == 0)
				yield return new CoroutineJob.ProgressInfo("ObiCloth: generating particles...",i/(float)topology.heVertices.Length);
		}
		
		// Generate deformable triangles:
		for (int i = 0; i < topology.heFaces.Length; i++){

			Oni.Face face = topology.heFaces[i];
			
			Oni.HalfEdge e1 = topology.heHalfEdges[face.halfEdge];
			Oni.HalfEdge e2 = topology.heHalfEdges[e1.nextHalfEdge];
			Oni.HalfEdge e3 = topology.heHalfEdges[e2.nextHalfEdge];

			deformableTriangles[i*3] = e1.endVertex;
			deformableTriangles[i*3+1] = e2.endVertex;
			deformableTriangles[i*3+2] = e3.endVertex;

			if (i % 500 == 0)
				yield return new CoroutineJob.ProgressInfo("ObiCloth: generating deformable geometry...",i/(float)topology.heFaces.Length);
		}

		List<ObiMeshTopology.HEEdge> edges = topology.GetEdgeList();

		DistanceConstraints.Clear();
		ObiDistanceConstraintBatch distanceBatch = new ObiDistanceConstraintBatch(true,false);
		DistanceConstraints.AddBatch(distanceBatch);

		// Create distance springs: 
		for (int i = 0; i < edges.Count; i++){
		
			Oni.HalfEdge hedge = topology.heHalfEdges[edges[i].halfEdgeIndex];
			Oni.Vertex startVertex = topology.heVertices[topology.GetHalfEdgeStartVertex(hedge)];
			Oni.Vertex endVertex = topology.heVertices[hedge.endVertex];
			
			distanceBatch.AddConstraint(topology.GetHalfEdgeStartVertex(hedge),hedge.endVertex,Vector3.Distance(startVertex.position,endVertex.position),1,1);		

			if (i % 500 == 0)
				yield return new CoroutineJob.ProgressInfo("ObiCloth: generating structural constraints...",i/(float)topology.heHalfEdges.Length);
		}

		// Cook distance constraints, for better cache and SIMD use:
		distanceBatch.Cook();
		
		// Create aerodynamic constraints:
		AerodynamicConstraints.Clear();
		ObiAerodynamicConstraintBatch aeroBatch = new ObiAerodynamicConstraintBatch(false,false);
		AerodynamicConstraints.AddBatch(aeroBatch);

		for (int i = 0; i < topology.heVertices.Length; i++){

			aeroBatch.AddConstraint(i,
									areaContribution[i],
			                        AerodynamicConstraints.dragCoefficient,
			                        AerodynamicConstraints.liftCoefficient);

			if (i % 500 == 0)
				yield return new CoroutineJob.ProgressInfo("ObiCloth: generating aerodynamic constraints...",i/(float)topology.heFaces.Length);
		}

		//Create skin constraints (if needed)
		if (skinnedMeshRenderer != null){

			SkinConstraints.Clear();
			ObiSkinConstraintBatch skinBatch = new ObiSkinConstraintBatch(true,false);
			SkinConstraints.AddBatch(skinBatch);

			for (int i = 0; i < topology.heVertices.Length; ++i){

				skinBatch.AddConstraint(i,topology.heVertices[i].position,
											   Vector3.up,0.05f,0.1f,0,1);

				if (i % 500 == 0)
					yield return new CoroutineJob.ProgressInfo("ObiCloth: generating skin constraints...",i/(float)topology.heVertices.Length);
			}

			for (int i = 0; i < topology.normals.Length; ++i){
				skinBatch.skinNormals[topology.visualMap[i]] = topology.normals[i];
			}

			skinBatch.Cook();

		}

		//Create pressure constraints if the mesh is closed:
		VolumeConstraints.Clear();

		if (skinnedMeshRenderer == null && topology.IsClosed){

			ObiVolumeConstraintBatch volumeBatch = new ObiVolumeConstraintBatch(false,false);
			VolumeConstraints.AddBatch(volumeBatch);

			int[] triangleIndices = new int[topology.heFaces.Length * 3];
			for (int i = 0; i < topology.heFaces.Length; i++){
				Oni.Face face = topology.heFaces[i];
			
				Oni.HalfEdge e1 = topology.heHalfEdges[face.halfEdge];
				Oni.HalfEdge e2 = topology.heHalfEdges[e1.nextHalfEdge];
				Oni.HalfEdge e3 = topology.heHalfEdges[e2.nextHalfEdge];

				triangleIndices[i*3] = e1.endVertex;
				triangleIndices[i*3+1] = e2.endVertex;
				triangleIndices[i*3+2] = e3.endVertex;

				if (i % 500 == 0)
					yield return new CoroutineJob.ProgressInfo("ObiCloth: generating volume constraints...",i/(float)topology.heFaces.Length);
			}

			volumeBatch.AddConstraint(triangleIndices,topology.MeshVolume,1,1);
		}
		
		//Create bending constraints:
		BendingConstraints.Clear();
		ObiBendConstraintBatch bendBatch = new ObiBendConstraintBatch(true,false);
		BendingConstraints.AddBatch(bendBatch);

		Dictionary<int,int> cons = new Dictionary<int, int>();
		for (int i = 0; i < topology.heVertices.Length; i++){
	
			Oni.Vertex vertex = topology.heVertices[i];
	
			foreach (Oni.Vertex n1 in topology.GetNeighbourVerticesEnumerator(vertex)){
	
				float cosBest = 0;
				Oni.Vertex vBest = n1;
	
				foreach (Oni.Vertex n2 in topology.GetNeighbourVerticesEnumerator(vertex)){
					float cos = Vector3.Dot((n1.position-vertex.position).normalized,
					                        (n2.position-vertex.position).normalized);
					if (cos < cosBest){
						cosBest = cos;
						vBest = n2;
					}
				}
				
				if (!cons.ContainsKey(vBest.index) || cons[vBest.index] != n1.index){
				
					cons[n1.index] = vBest.index;
				
					float[] restPositions = new float[]{n1.position[0],n1.position[1],n1.position[2],
														vBest.position[0],vBest.position[1],vBest.position[2],
														vertex.position[0],vertex.position[1],vertex.position[2]};
					float restBend = Oni.BendingConstraintRest(restPositions);
					bendBatch.AddConstraint(n1.index,vBest.index,vertex.index,restBend,0,1);
				}
	
			}
	
			if (i % 500 == 0)
				yield return new CoroutineJob.ProgressInfo("ObiCloth: adding bend constraints...",i/(float)sharedTopology.heVertices.Length);
		}

		bendBatch.Cook();

		// Initialize tether constraints:
		TetherConstraints.Clear();

		// Initialize pin constraints:
		PinConstraints.Clear();
		ObiPinConstraintBatch pinBatch = new ObiPinConstraintBatch(false,false);
		PinConstraints.AddBatch(pinBatch);

		AddToSolver(null);

		initializing = false;
		initialized = true;

		if (skinnedMeshRenderer == null)
			InitializeWithRegularMesh();
		else 
			InitializeWithSkinnedMesh();
	}

	/**
	 * In the case of skinned cloth, we must grab the skinned vertex positions before starting the simulation steps for this frame.
	 * If the Animator component is set to Animate Physics, then we'll get up to date data here. Otherwise, we will suffer a 1 frame delay
	 * as the animation and simulation update at the same time (instead of first animation, then simulation which is the optimal way).
	 */
	public override void OnSolverStepBegin(){
		if (skinnedMeshRenderer == null){
			// regular on step begin: transform fixed particles.
			base.OnSolverStepBegin();
		}else{

			// manually update animator (before particle physics):
			if (animator != null && animator.enabled){
					animator.StopPlayback();
				animator.Update(Time.fixedDeltaTime);
			}

			// apply world space velocity:
			ApplyWorldSpaceVelocity();

			// grab skeleton bone transforms:
			GrabSkeletonBones();
		}
	}

	public void ApplyWorldSpaceVelocity(){

		if (!Solver.simulateInLocalSpace || worldVelocityScale == 0 || !this.enabled) 
			return;

		Matrix4x4 delta = Solver.transform.worldToLocalMatrix * Solver.LastTransform;
		Vector4[] simulationPosition = {Vector4.zero};

		for(int i = 0; i < particleIndices.Length; i++){
			if (invMasses[i] > 0){
				Oni.GetParticlePositions(solver.OniSolver,simulationPosition,1,particleIndices[i]);
				simulationPosition[0] = Vector3.Lerp(simulationPosition[0],delta.MultiplyPoint3x4(simulationPosition[0]),worldVelocityScale);
				Oni.SetParticlePositions(solver.OniSolver,simulationPosition,1,particleIndices[i]);
			}
		}
	}

	/**
	 * If a Skinned Mesh Renderer is present, grabs all mesh data from the current animation state and transfers it to the particle simulation.
	 * Does nothing if no Skinned Mesh Renderer can be found.
	 */
	public void GrabSkeletonBones(){

		if (skinnedMeshRenderer != null){

			if (!Initialized || clothMesh == null || particleIndices == null) return;

			Transform[] rendererBones = skinnedMeshRenderer.bones;
			float[] bones = new float[16*rendererBones.Length];
			
			for (int p = 0; p < sharedMesh.bindposes.Length; ++p){
	
				Matrix4x4 bone;

				if (Solver.simulateInLocalSpace)
					bone = Solver.transform.worldToLocalMatrix * rendererBones[p].localToWorldMatrix;
				else 
					bone = rendererBones[p].localToWorldMatrix;

				for (int i = 0; i < 16; ++i)
					bones[p*16+i] = bone[i];
			}

			Oni.SetDeformableMeshBoneTransforms(deformableMesh,bones);
		}

	}

	/**
	 * Automatically generates tether constraints for the cloth.
	 * Partitions fixed particles into "islands", then generates up to maxTethers constraints for each 
	 * particle, linking it to the closest point in each island.
	 */
	public override bool GenerateTethers(int maxTethers){
		
		if (!Initialized) return false;

		TetherConstraints.Clear();
		
		if (maxTethers > 0){

			ObiTetherConstraintBatch tetherBatch = new ObiTetherConstraintBatch(true,false);
			TetherConstraints.AddBatch(tetherBatch);
			
			List<HashSet<int>> islands = new List<HashSet<int>>();
			
			// Partition fixed particles into islands:
			for (int i = 0; i < topology.heVertices.Length; i++){
				
				Oni.Vertex vertex = topology.heVertices[i];
				if (invMasses[i] > 0 || !active[i]) continue;
				
				int assignedIsland = -1;

				// keep a list of islands to merge with ours:
				List<int> mergeableIslands = new List<int>();
					
				// See if any of our neighbors is part of an island:
				foreach (Oni.Vertex n in topology.GetNeighbourVerticesEnumerator(vertex)){

					if (!active[n.index]) continue;
		
					for(int k = 0; k < islands.Count; ++k){

						if (islands[k].Contains(n.index)){

							// if we are not in an island yet, pick this one:
							if (assignedIsland < 0){
								assignedIsland = k;
	                            islands[k].Add(i);
							}
							// if we already are in an island, we will merge this newfound island with ours:
							else if (assignedIsland != k && !mergeableIslands.Contains(k)){
								mergeableIslands.Add(k);
							}
						}
                    }
				}

				// merge islands with the assigned one:
				foreach(int merge in mergeableIslands){
					islands[assignedIsland].UnionWith(islands[merge]);
				}

				// remove merged islands:
				mergeableIslands.Sort();
				mergeableIslands.Reverse();
				foreach(int merge in mergeableIslands){
					islands.RemoveAt(merge);
				}
				
				// If no adjacent particle is in an island, create a new one:
				if (assignedIsland < 0){
					islands.Add(new HashSet<int>(){i});
				}
			}	
			
			// Generate tether constraints:
			for (int i = 0; i < invMasses.Length; ++i){
			
				if (invMasses[i] == 0 || !active[i]) continue;
				
				List<KeyValuePair<float,int>> tethers = new List<KeyValuePair<float,int>>(islands.Count*maxTethers);
				
				// Find the closest particle in each island, and add it to tethers.
				foreach(HashSet<int> island in islands){
					int closest = -1;
					float minDistance = Mathf.Infinity;
					foreach (int j in island){
						float distance = (topology.heVertices[i].position - topology.heVertices[j].position).sqrMagnitude;
						if (distance < minDistance){
							minDistance = distance;
							closest = j;
						}
					}
					if (closest >= 0)
						tethers.Add(new KeyValuePair<float,int>(minDistance, closest));
				}
				
				// Sort tether indices by distance:
				tethers.Sort(
				delegate(KeyValuePair<float,int> x, KeyValuePair<float,int> y)
				{
					return x.Key.CompareTo(y.Key);
				}
				);
				
				// Create constraints for "maxTethers" closest anchor particles:
				for (int k = 0; k < Mathf.Min(maxTethers,tethers.Count); ++k){
					tetherBatch.AddConstraint(i,tethers[k].Value,Mathf.Sqrt(tethers[k].Key),
												TetherConstraints.tetherScale,
												TetherConstraints.stiffness);
				}
			}
            
			tetherBatch.Cook();
        }
        
        return true;
        
	}
}
}

