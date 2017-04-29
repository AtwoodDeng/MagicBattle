using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;

namespace Obi{

/**
 * ObiColliderGroup holds references to all colliders and rigidbodies that any given ObiSolver should be aware of. Multiple ObiSolvers
 * can share a single ObiColliderGroup.
 */
[ExecuteInEditMode]
public class ObiColliderGroup : MonoBehaviour
{

	public class MeshColliderShapeAndData{
		public Oni.TriangleMeshData meshData;
		public Oni.TriangleMeshShape meshShape;
		public int shapeIndex;

		public MeshColliderShapeAndData(Oni.TriangleMeshData meshData, Oni.TriangleMeshShape meshShape, int shapeIndex)
		{
			this.meshData = meshData;
			this.meshShape = meshShape;
			this.shapeIndex = shapeIndex;
		}
	}

	public class EdgeColliderShapeAndData{
		public Oni.EdgeMeshData meshData;
		public Oni.EdgeMeshShape meshShape;
		public int shapeIndex;

		public EdgeColliderShapeAndData(Oni.EdgeMeshData meshData, Oni.EdgeMeshShape meshShape, int shapeIndex)
		{
			this.meshData = meshData;
			this.meshShape = meshShape;
			this.shapeIndex = shapeIndex;
		}
	}

	[SerializeField] private List<Collider> colliders = new List<Collider>();
	[SerializeField] private List<Collider2D> colliders2D = new List<Collider2D>();

	[HideInInspector]public IntPtr oniColliderGroup;

	Dictionary<int,int> rigidbodyIDs = new Dictionary<int,int>();  /**<holds pairs of <instanceid,index in oniRigidbodies>, to help with rigidbody assignment.*/

	[NonSerialized] public Dictionary <TerrainCollider,Oni.HeightData> heightData = new Dictionary<TerrainCollider,Oni.HeightData>();
	[NonSerialized] public Dictionary <Mesh,MeshColliderShapeAndData> meshColliderData = new Dictionary<Mesh,MeshColliderShapeAndData>();	
	[NonSerialized] public Dictionary <EdgeCollider2D,EdgeColliderShapeAndData> edgeColliderData = new Dictionary<EdgeCollider2D,EdgeColliderShapeAndData>();

	bool hasBeenUpdated = false;	/**< whether the group has been updated this frame or not.*/
	bool usedInLocalSpace = false;  /**< whether the group is used by any solver simulating in local space.*/
	[HideInInspector][SerializeField] bool colliderListsHaveChanged = false;

	public IEnumerable<Collider> Colliders
	{
		get{return colliders.AsReadOnly();}
	}

	public IEnumerable<Collider2D> Colliders2D
	{
		get{return colliders2D.AsReadOnly();}
	}

	public int ColliderCount{
		get{return colliders.Count;}
	}

	public int Collider2DCount{
		get{return colliders2D.Count;}
	}

	void OnEnable(){
		oniColliderGroup = Oni.CreateColliderGroup();
	}
	
	void OnDisable(){
		Oni.DestroyColliderGroup(oniColliderGroup);
		
		foreach(Oni.HeightData data in heightData.Values){
			data.UnpinData();
		}
		heightData.Clear();

		foreach(MeshColliderShapeAndData data in meshColliderData.Values){
			data.meshData.UnpinData();
		}
		meshColliderData.Clear();

		foreach(EdgeColliderShapeAndData data in edgeColliderData.Values){
			data.meshData.UnpinData();
		}
		edgeColliderData.Clear();
    }

	public void ForcePinConstraintsUpdate(){
		colliderListsHaveChanged = true;
	}

	public void AddCollider(Collider c){
		if (c != null){
			colliders.Add(c);
			colliderListsHaveChanged = true;
		}
	}

	public bool RemoveCollider(Collider c){
		colliderListsHaveChanged = true;
		return colliders.Remove(c);
	}

	public void AddCollider2D(Collider2D c){
		if (c != null){
			colliders2D.Add(c);
			colliderListsHaveChanged = true;
		}
	}

	public bool RemoveCollider2D(Collider2D c){
		colliderListsHaveChanged = true;
		return colliders2D.Remove(c);
	}

	public int GetIndexOfCollider(Collider c){
		if (colliders != null && c != null){
			return colliders.IndexOf(c);
		}
		return -1;
	}

	public int GetIndexOfCollider2D(Collider c){
		if (colliders != null && c != null){
			return colliders.IndexOf(c);
		}
		return -1;
	}

	public void UpdateTerrainHeightInfo(TerrainCollider terrain){
		Oni.HeightData height;
		if (heightData.TryGetValue(terrain, out height)){
			height.UpdateHeightData();
		}
	}	

	public void UpdateMeshColliderTriangleInfo(MeshCollider mesh){
		MeshColliderShapeAndData shapeAndData;
		if (mesh.sharedMesh != null && meshColliderData.TryGetValue(mesh.sharedMesh, out shapeAndData)){
			Oni.UpdateTriangleMeshShapes(oniColliderGroup,1,shapeAndData.shapeIndex);
		}
	}

	public void UpdateMeshColliderTriangleInfo(EdgeCollider2D edgeCollider){
		EdgeColliderShapeAndData shapeAndData;
		if (edgeColliderData.TryGetValue(edgeCollider, out shapeAndData)){
			Oni.UpdateTriangleMeshShapes(oniColliderGroup,1,shapeAndData.shapeIndex);
		}
	}

	public void UpdateBodiesInfo(ObiSolver solver){

		usedInLocalSpace |= solver.simulateInLocalSpace;

		if (hasBeenUpdated){ 
			if (usedInLocalSpace){
				Debug.LogWarning("ObiColliderGroups used by ObiSolvers simulating in local space cannot be shared by multiple solvers."+
								 "Please duplicate the ObiColliderGroup if you want to use it in other solvers.");
			}
			return;
		}		
	
		hasBeenUpdated = true;

		Oni.RemoveColliders(oniColliderGroup,-1,0);
		Oni.RemoveRigidbodies(oniColliderGroup,-1,0);
		Oni.RemoveSphereShapes(oniColliderGroup,-1,0);
		Oni.RemoveCapsuleShapes(oniColliderGroup,-1,0);
		Oni.RemoveBoxShapes(oniColliderGroup,-1,0);
		Oni.RemoveHeightmapShapes(oniColliderGroup,-1,0);
		//Oni.RemoveTriangleMeshShapes(oniColliderGroup,-1,0);

		rigidbodyIDs.Clear();

		UpdateColliders(solver);
		Update2DColliders();

		if (colliderListsHaveChanged){
			UpdatePinConstraints(solver);
			colliderListsHaveChanged = false;
		}
	}

	/**
	 * Updates pin constraint collider indices for a given solver.
	 */
	private void UpdatePinConstraints(ObiSolver solver){
		
		if (solver == null) return;

		// for all actors in the solver:
		foreach(ObiActor actor in solver.actors){

			// See if they use pin contraints:
			ObiPinConstraints pinConstraints = actor.GetComponent<ObiPinConstraints>();
			if (pinConstraints != null){	

				// for each constraint batch, update collider indices.
				foreach(ObiPinConstraintBatch batch in pinConstraints.GetBatches()){
					batch.UpdateColliderIndices(this);
				}
		
				// push new data to the solver:
				pinConstraints.PushDataToSolver();
			}
		}
	}

	private void UpdateColliders(ObiSolver solver){

		for(int i = 0; i < colliders.Count; i++)
		{
			Collider source = colliders[i];
			if (source == null || !source.enabled || !source.gameObject.activeInHierarchy) continue;

			Rigidbody rb = source.GetComponentInParent<Rigidbody>();
			ObiCollider oc = source.GetComponent<ObiCollider>();			

			// Get the adequate rigidBodyIndex. If several colliders share a rigidbody, they'll get the same rigidBodyIndex.
			int rigidBodyIndex = -1;
			if (rb != null){

				if (!rigidbodyIDs.TryGetValue(rb.GetInstanceID(),out rigidBodyIndex)){

					ObiRigidbody or = rb.GetComponent<ObiRigidbody>();

					rigidBodyIndex = Oni.GetRigidbodyCount(oniColliderGroup);
					Oni.SetRigidbodies(oniColliderGroup,new Oni.Rigidbody[]{
							new Oni.Rigidbody(rb,(or != null) ? or.kinematicForParticles : false)
					},1,rigidBodyIndex);
					rigidbodyIDs[rb.GetInstanceID()] = rigidBodyIndex;

				}

			}

			float thickness = (oc != null)?oc.thickness:0;
			Oni.Collider collider = new Oni.Collider();
			bool supported = true;

			if (source is SphereCollider){
				collider = new Oni.Collider(source,Oni.ShapeType.Sphere,thickness,Oni.GetShapeCount(oniColliderGroup,Oni.ShapeType.Sphere),rigidBodyIndex,(oc != null)?oc.materialIndex:0);
				
				Oni.SetSphereShapes(oniColliderGroup,new Oni.SphereShape[]{
					new Oni.SphereShape(source as SphereCollider)
				},1,Oni.GetShapeCount(oniColliderGroup,Oni.ShapeType.Sphere));
			}else if (source is BoxCollider){
				collider = new Oni.Collider(source,Oni.ShapeType.Box,thickness,Oni.GetShapeCount(oniColliderGroup,Oni.ShapeType.Box),rigidBodyIndex,(oc != null)?oc.materialIndex:0);

				Oni.SetBoxShapes(oniColliderGroup,new Oni.BoxShape[]{
					new Oni.BoxShape(source as BoxCollider)
				},1,Oni.GetShapeCount(oniColliderGroup,Oni.ShapeType.Box));
			}else if (source is CapsuleCollider){
				collider = new Oni.Collider(source,Oni.ShapeType.Capsule,thickness,Oni.GetShapeCount(oniColliderGroup,Oni.ShapeType.Capsule),rigidBodyIndex,(oc != null)?oc.materialIndex:0);

				Oni.SetCapsuleShapes(oniColliderGroup,new Oni.CapsuleShape[]{
					new Oni.CapsuleShape(source as CapsuleCollider)
				},1,Oni.GetShapeCount(oniColliderGroup,Oni.ShapeType.Capsule));
			}else if (source is CharacterController){
				collider = new Oni.Collider(source,Oni.ShapeType.Capsule,thickness,Oni.GetShapeCount(oniColliderGroup,Oni.ShapeType.Capsule),rigidBodyIndex,(oc != null)?oc.materialIndex:0);
				
				Oni.SetCapsuleShapes(oniColliderGroup,new Oni.CapsuleShape[]{
					new Oni.CapsuleShape(source as CharacterController)
				},1,Oni.GetShapeCount(oniColliderGroup,Oni.ShapeType.Capsule));
			}else if (source is TerrainCollider){

				TerrainCollider tc = source as TerrainCollider;

				Oni.HeightData data;
				if (!heightData.TryGetValue(tc,out data)){
					data = heightData[tc] = new Oni.HeightData(tc);
				}
	
				collider = new Oni.Collider(source,Oni.ShapeType.Heightmap,thickness,Oni.GetShapeCount(oniColliderGroup,Oni.ShapeType.Heightmap),rigidBodyIndex,(oc != null)?oc.materialIndex:0);
				
				Oni.SetHeightmapShapes(oniColliderGroup,new Oni.HeightmapShape[]{
					new Oni.HeightmapShape(tc,data.AddrOfHeightData())
				},1,Oni.GetShapeCount(oniColliderGroup,Oni.ShapeType.Heightmap));
				
			}else if (source is MeshCollider){

				MeshCollider mc = source as MeshCollider;
				MeshColliderShapeAndData shapeAndData;

				if (mc.sharedMesh != null){

					// We can share the same triangle data across several instances of the same MeshCollider:
					if (!meshColliderData.TryGetValue(mc.sharedMesh,out shapeAndData)){
	
						// Get current amount of triangle mesh shapes:
						int shapeIndex = Oni.GetShapeCount(oniColliderGroup,Oni.ShapeType.TriangleMesh);
	
						// Generate mesh collider triangle data and shape:
						Oni.TriangleMeshData data = new Oni.TriangleMeshData(mc);
							Oni.TriangleMeshShape shape = new Oni.TriangleMeshShape(mc,
																					(oc != null)?oc.meshColliderType:Oni.TriangleMeshShape.MeshColliderType.ThinTwoSided,
																					thickness,
																					data.AddrOfVertexData(),
																				    data.AddrOfTriangleData());
	
						// Pack both in a small wrapper:
						shapeAndData = new MeshColliderShapeAndData(data, shape, shapeIndex);
	
						// Tell Oni we want to define a new triangle mesh:
						Oni.SetTriangleMeshShapes(oniColliderGroup,
												  new Oni.TriangleMeshShape[]{shapeAndData.meshShape},
												  1,shapeIndex);
	
						Oni.UpdateTriangleMeshShapes(oniColliderGroup,1,shapeIndex);
	
						// Store mesh collider data:
						meshColliderData[mc.sharedMesh] = shapeAndData;
					}
		
					collider = new Oni.Collider(source,Oni.ShapeType.TriangleMesh,thickness,shapeAndData.shapeIndex,rigidBodyIndex,(oc != null)?oc.materialIndex:0);
					
				}
				
			}else{
				supported = false;
				Debug.LogWarning("Collider type "+source.GetType()+" not supported by Obi. Ignoring it.");
			}

			if (supported){
			
				// convert to colliders to solver's local space:
				if (solver.simulateInLocalSpace) 
					collider.SetSpaceTransform(solver.transform);

				Oni.SetColliders(oniColliderGroup,new Oni.Collider[]{collider},1,Oni.GetColliderCount(oniColliderGroup));
			}
			
		}
	}

	private void Update2DColliders(){

		for(int i = 0; i < colliders2D.Count; i++)
		{
			Collider2D source = colliders2D[i];
			if (source == null || !source.enabled || !source.gameObject.activeInHierarchy) continue;

			Rigidbody2D rb = source.GetComponentInParent<Rigidbody2D>();
			ObiCollider oc = source.GetComponent<ObiCollider>();			

			// Get the adequate rigidBodyIndex. If several colliders share a rigidbody, they'll get the same rigidBodyIndex.
			int rigidBodyIndex = -1;
			if (rb != null){

				if (!rigidbodyIDs.TryGetValue(rb.GetInstanceID(),out rigidBodyIndex)){

					ObiRigidbody or = rb.GetComponent<ObiRigidbody>();

					rigidBodyIndex = Oni.GetRigidbodyCount(oniColliderGroup);
					Oni.SetRigidbodies(oniColliderGroup,new Oni.Rigidbody[]{
						new Oni.Rigidbody(rb,(or != null) ? or.kinematicForParticles : false)
					},1,rigidBodyIndex);
					rigidbodyIDs[rb.GetInstanceID()] = rigidBodyIndex;

				}

			}

			float thickness = (oc != null)?oc.thickness:0;
	
			if (source is CircleCollider2D){
				Oni.SetColliders(oniColliderGroup,new Oni.Collider[]{
					new Oni.Collider(source,Oni.ShapeType.Sphere,thickness,Oni.GetShapeCount(oniColliderGroup,Oni.ShapeType.Sphere),rigidBodyIndex,(oc != null)?oc.materialIndex:0)
				},1,Oni.GetColliderCount(oniColliderGroup));
				Oni.SetSphereShapes(oniColliderGroup,new Oni.SphereShape[]{
					new Oni.SphereShape(source as CircleCollider2D)
				},1,Oni.GetShapeCount(oniColliderGroup,Oni.ShapeType.Sphere));
			}else if (source is BoxCollider2D){
				Oni.SetColliders(oniColliderGroup,new Oni.Collider[]{
					new Oni.Collider(source,Oni.ShapeType.Box,thickness,Oni.GetShapeCount(oniColliderGroup,Oni.ShapeType.Box),rigidBodyIndex,(oc != null)?oc.materialIndex:0)
				},1,Oni.GetColliderCount(oniColliderGroup));
				Oni.SetBoxShapes(oniColliderGroup,new Oni.BoxShape[]{
					new Oni.BoxShape(source as BoxCollider2D)
				},1,Oni.GetShapeCount(oniColliderGroup,Oni.ShapeType.Box));
			}else if (source is EdgeCollider2D){

				EdgeCollider2D mc = source as EdgeCollider2D;
				EdgeColliderShapeAndData shapeAndData;

				if (!edgeColliderData.TryGetValue(source as EdgeCollider2D,out shapeAndData)){

					// Get current amount of triangle mesh shapes:
					int shapeIndex = Oni.GetShapeCount(oniColliderGroup,Oni.ShapeType.EdgeMesh);

					// Generate mesh collider triangle data and shape:
					Oni.EdgeMeshData data = new Oni.EdgeMeshData(mc);
					Oni.EdgeMeshShape shape = new Oni.EdgeMeshShape(mc,
																	thickness,
																	data.AddrOfVertexData(),
																    data.AddrOfEdgeData());

					// Pack both in a small wrapper:
					shapeAndData = new EdgeColliderShapeAndData(data, shape, shapeIndex);

					// Tell Oni we want to define a new triangle mesh:
					Oni.SetEdgeMeshShapes(oniColliderGroup,
										  new Oni.EdgeMeshShape[]{shapeAndData.meshShape},
										  1,shapeIndex);

					Oni.UpdateEdgeMeshShapes(oniColliderGroup,1,shapeIndex);

					// Store mesh collider data:
					edgeColliderData[mc] = shapeAndData;
				}
	
				Oni.SetColliders(oniColliderGroup,new Oni.Collider[]{
					new Oni.Collider(source,Oni.ShapeType.EdgeMesh,thickness,shapeAndData.shapeIndex,rigidBodyIndex,(oc != null)?oc.materialIndex:0)
				},1,Oni.GetColliderCount(oniColliderGroup));
				
			}else{
				Debug.LogWarning("2D Collider type "+source.GetType()+" not supported by Obi. Ignoring it.");
			}
			
		}
	}	

	/**
	 * Injects back into the rigidbodies the new velocities calculated by the solver in response to patrticle interactions:
 	 */
	public void UpdateVelocities(){ 

		if (!hasBeenUpdated) return;
			hasBeenUpdated = false;

		// Collider groups used by solvers simulating in local space do not support two-way interaction:
		if (usedInLocalSpace)
			return;

		usedInLocalSpace = false;

		Oni.Rigidbody[] bodies = new Oni.Rigidbody[rigidbodyIDs.Count];
		Oni.GetRigidbodies(oniColliderGroup,bodies,rigidbodyIDs.Count,0);

		// Update 3d collider velocities:
		for (int i = 0; i < colliders.Count; ++i){

			Collider collider = colliders[i];
			if (collider == null || !collider.enabled || !collider.gameObject.activeInHierarchy) continue;

			int rigidBodyIndex = -1;
			Rigidbody rb = collider.GetComponentInParent<Rigidbody>();

			if (rb != null && rigidbodyIDs.TryGetValue(rb.GetInstanceID(),out rigidBodyIndex)){
				if (rigidBodyIndex < bodies.Length && bodies[rigidBodyIndex].impulseCount > 0){
					rb.velocity = bodies[rigidBodyIndex].linearVelocity;
					rb.angularVelocity = bodies[rigidBodyIndex].angularVelocity;	
				}
			}

		}

		// Update 2d collider velocities:
		for (int i = 0; i < colliders2D.Count; ++i){

			Collider2D collider = colliders2D[i];
			if (collider == null || !collider.enabled || !collider.gameObject.activeInHierarchy) continue;

			int rigidBodyIndex = -1;
			Rigidbody2D rb = collider.GetComponentInParent<Rigidbody2D>();

			if (rb != null && rigidbodyIDs.TryGetValue(rb.GetInstanceID(),out rigidBodyIndex)){
				if (rigidBodyIndex < bodies.Length && bodies[rigidBodyIndex].impulseCount > 0){
					rb.velocity = bodies[rigidBodyIndex].linearVelocity;

					// For some weird reason, in 2D angular velocity is measured in *degrees* per second,
				 	// instead of radians/second. Seriously Unity, WTF??
					rb.angularVelocity = bodies[rigidBodyIndex].angularVelocity.z * Mathf.Rad2Deg;
				}
			}

		}

	}

}
}

