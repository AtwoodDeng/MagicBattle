using UnityEngine;
using System;
using System.Collections;
using System.Runtime.InteropServices;
using Obi;

/**
 * Interface for the Oni particle physics library.
 */
public static class Oni {

	public enum ConstraintType
    {
        Tether = 0,
        Pin = 1,
        Volume = 2,
        Bending = 3,
        Distance = 4,
        ParticleCollision = 5,
        Density = 6,
        Collision = 7,
        Skin = 8,
        Aerodynamics = 9,
        ShapeMatching = 10,
    };

	public enum ParticlePhase{
		SelfCollide = 1 << 24,
		Fluid = 1 << 25
	}

	public enum ShapeType{
		Sphere = 0,
		Box = 1,
		Capsule = 2,
		Heightmap = 3,
		TriangleMesh = 4,
		EdgeMesh = 5
	}

	public enum MaterialCombineMode{
		Average = 0,
		Minimium = 1,
		Multiply = 2,
        Maximum = 3
    }

	public enum NormalsUpdate{
		Recalculate = 0,
		Skin = 1
	}

	[StructLayout(LayoutKind.Sequential, Pack = 1)]
	public struct ProfileInfo
	{
		public double start;
		public double end;
		public double childDuration;
		public int threadID;
		public int level;
		[MarshalAs(UnmanagedType.ByValTStr, SizeConst = 64)]
		public string name;
	}

	[Serializable]
	[StructLayout(LayoutKind.Sequential, Pack = 1)]
	public struct SolverParameters{

		public enum Interpolation
		{
			None,
			Interpolate,
		};

		public enum Mode
		{
			Mode3D,
			Mode2D,
		};

		[Tooltip("In 2D mode, particles are simulated on the XY plane only. For use in conjunction with Unity's 2D mode.")]
		public Mode mode;

		[Tooltip("Same as Rigidbody.interpolation. Set to INTERPOLATE for cloth that is applied on a main character or closely followed by a camera. NONE for everything else.")]
		public Interpolation interpolation;

		public Vector3 gravity;

		[Tooltip("Percentage of velocity lost per second, between 0% (0) and 100% (1).")]
		[Range(0,1)]
		public float damping; 

		[Tooltip("Radius of diffuse particle advection. Large values yield better quality but are more expensive.")]
		public float advectionRadius; 	

		[Tooltip("Kinetic energy below which particle positions arent updated. Energy values are mass-normalized, so all particles in the solver have the same threshold.")]
		public float sleepThreshold; 		              		              

		public SolverParameters(Interpolation interpolation, Vector4 gravity){
			this.mode = Mode.Mode3D;
			this.gravity = gravity;
			this.interpolation = interpolation;
			damping = 0;
			advectionRadius = 0.5f;
			sleepThreshold = 0.001f;
		}

	}

	[Serializable]
	[StructLayout(LayoutKind.Sequential, Pack = 1)]
	public struct ConstraintParameters{

		public enum EvaluationOrder
		{
			Sequential,
			Parallel
		};

		[Tooltip("Whether this constraint group is solved or not.")]
		[MarshalAs(UnmanagedType.I1)]
		public bool enabled;

		[Tooltip("Order in which constraints are evaluated. SEQUENTIAL converges faster but is not very stable. PARALLEL is very stable but converges slowly, requiring more iterations to achieve the same result.")]
		public EvaluationOrder evaluationOrder;								/**< Constraint evaluation order.*/
		
		[Tooltip("Number of relaxation iterations performed by the constraint solver. A low number of iterations will perform better, but be less accurate.")]
		public int iterations;												/**< Amount of solver iterations per step for this constraint group.*/
		
		[Tooltip("Over (or under if < 1) relaxation factor used. At 1, no overrelaxation is performed. At 2, constraints double their relaxation rate. High values reduce stability but improve convergence.")]
		[Range(0.1f,2)]
		public float SORFactor;												/**< Sucessive over-relaxation factor for parallel evaluation order.*/
		

		public ConstraintParameters(bool enabled, EvaluationOrder order, int iterations){
			this.enabled = enabled;
			this.iterations = iterations;
			this.evaluationOrder = order;
			this.SORFactor = 1;
		}
		
	}

	// In this particular case, size is forced to 64 bytes to ensure 16 byte memory alignment needed by Oni.
	[StructLayout(LayoutKind.Sequential, Pack = 1, Size = 64)]
	public struct Contact{
		public Vector4 point; 		   /**< Speculative point of contact for each collision. */
        public Vector4 normal;		   /**< Normal direction for each collision. */
        public float distance;    /** distance between both colliding entities at the beginning of the timestep.*/
        
        public float normalImpulse;
        public float tangentImpulse;
        public float stickImpulse;
        
        public int particle; /** particle index*/
        public int other;    /** particle or rigidbody index*/
	}

	[Serializable]
	[StructLayout(LayoutKind.Sequential, Pack = 1)]
	public struct Bounds{
		public Vector4 min;
		public Vector4 max;

		public Vector4 Center{
			get{return min + (max-min)*0.5f;}
		}

		public Vector4 Size{
			get{return max-min;}
		}

		public Bounds(Vector4 min, Vector4 max){
			this.min = min;
			this.max = max;
		}

		public void Transform(Matrix4x4 m)
		{
		    var xa = m.GetColumn(0) * min.x;
		    var xb = m.GetColumn(0) * max.x;
		 
		    var ya = m.GetColumn(1) * min.y;
		    var yb = m.GetColumn(1) * max.y;
		 
		    var za = m.GetColumn(2) * min.z;
		    var zb = m.GetColumn(2) * max.z;
		 
			Vector3 pos = m.GetColumn(3);
			min = Vector3.Min(xa, xb) + Vector3.Min(ya, yb) + Vector3.Min(za, zb) + pos;
			max = Vector3.Max(xa, xb) + Vector3.Max(ya, yb) + Vector3.Max(za, zb) + pos;

		}
	}

	[StructLayout(LayoutKind.Sequential, Pack = 1)]
	public struct BoneWeights
	{
		public int bone0;
		public int bone1;
		public int bone2;
		public int bone3;
		public float weight0;
		public float weight1;
		public float weight2;
		public float weight3;

		public BoneWeights(BoneWeight weight){
			bone0 = weight.boneIndex0;
			bone1 = weight.boneIndex1;
			bone2 = weight.boneIndex2;
			bone3 = weight.boneIndex3;
			weight0 = weight.weight0;
			weight1 = weight.weight1;
			weight2 = weight.weight2;
			weight3 = weight.weight3;
		}
	}

	// In this particular case, size is forced to 80 bytes to ensure 16 byte memory alignment needed by Oni.
	[Serializable]
	[StructLayout(LayoutKind.Sequential, Pack = 1, Size = 80)]
	public struct Rigidbody{

		public Vector4 linearVelocity;
		public Vector4 angularVelocity;
		public Vector4 centerOfMass;
		public Vector4 inertiaTensor;
		public float inverseMass;
		public int impulseCount;
		
		public Rigidbody(UnityEngine.Rigidbody source, bool kinematicForParticles){

			bool kinematic = source.isKinematic || kinematicForParticles;

			linearVelocity = kinematic ?  Vector3.zero : source.velocity;
			angularVelocity = kinematic ? Vector3.zero : source.angularVelocity;

			// center of mass in unity is affected by local rotation and poistion, but not scale. We need it expressed in world space:
			centerOfMass = source.transform.position + source.transform.rotation * source.centerOfMass;

			Vector3 invTensor = new Vector3((source.constraints & RigidbodyConstraints.FreezeRotationX) != 0?0:1/source.inertiaTensor.x,
											(source.constraints & RigidbodyConstraints.FreezeRotationY) != 0?0:1/source.inertiaTensor.y,
											(source.constraints & RigidbodyConstraints.FreezeRotationZ) != 0?0:1/source.inertiaTensor.z);

			Vector3 invTensorDiagonal = source.inertiaTensorRotation * invTensor;

			// the inertia tensor is a diagonal matrix (Vector3) because it is expressed in the space generated by the principal axes of rotation (inertiaTensorRotation).
			inertiaTensor = kinematic ? Vector4.zero : new Vector4(invTensorDiagonal.x,invTensorDiagonal.y,invTensorDiagonal.z,0);
			inverseMass = kinematic ? 0 : 1/source.mass;

			impulseCount = 0;
		}

		public Rigidbody(UnityEngine.Rigidbody2D source, bool kinematicForParticles){

			linearVelocity = source.velocity;

			// For some weird reason, in 2D angular velocity is measured in *degrees* per second, 
			// instead of radians. Seriously Unity, WTF??
			angularVelocity = new Vector4(0,0,source.angularVelocity * Mathf.Deg2Rad,0);

			// center of mass in unity is affected by local rotation and poistion, but not scale. We need it expressed in world space:
			centerOfMass = source.transform.position + source.transform.rotation * source.centerOfMass;

			inertiaTensor = (source.isKinematic || kinematicForParticles) ? Vector4.zero : new Vector4(0,0,(source.constraints & RigidbodyConstraints2D.FreezeRotation) != 0?0:1/source.inertia,0);
			inverseMass = (source.isKinematic || kinematicForParticles) ? 0 : 1/source.mass;

			impulseCount = 0;
		}
	}

	// In this particular case, size is forced to 128 bytes to ensure 16 byte memory alignment needed by Oni.
	[Serializable]
	[StructLayout(LayoutKind.Sequential, Pack = 1, Size = 128)]
	public struct Collider{
		public Oni.Bounds bounds;
		public Quaternion rotation;
		public Vector4 translation;
		public Vector4 scale;
		public float contactOffset;
		public int collisionGroup;
		public ShapeType shapeType;
		public int shapeIndex;
		public int rigidbodyIndex;
		public int materialIndex;

		[MarshalAs(UnmanagedType.LPArray, SizeConst=18)]
		float[] rotationCache;	// padding for the rotation cache.

		public Collider(UnityEngine.Collider source, ShapeType shapeType, float thickness, int shapeIndex, int rigidbodyIndex, int materialIndex){
			bounds = new Oni.Bounds(source.bounds.min - Vector3.one*(thickness + source.contactOffset),
									source.bounds.max + Vector3.one*(thickness + source.contactOffset));
			translation = source.transform.position;
			rotation = source.transform.rotation;
			scale = new Vector4(source.transform.lossyScale.x,source.transform.lossyScale.y,source.transform.lossyScale.z,1);
			contactOffset = thickness;
			this.collisionGroup = source.gameObject.layer;
			this.shapeType = shapeType;
			this.shapeIndex = shapeIndex;
			this.rigidbodyIndex = rigidbodyIndex;
			this.materialIndex = materialIndex;
			this.rotationCache = new float[16];
		}

		public Collider(UnityEngine.Collider2D source, ShapeType shapeType, float thickness, int shapeIndex, int rigidbodyIndex, int materialIndex){
			bounds = new Oni.Bounds(source.bounds.min - Vector3.one * (thickness + 0.01f), //allow some room for contacts to be generated before penetration.
									source.bounds.max + Vector3.one * (thickness + 0.01f));
			translation = source.transform.position;
			rotation = source.transform.rotation;
			scale = new Vector4(source.transform.lossyScale.x,source.transform.lossyScale.y,source.transform.lossyScale.z,1);
			contactOffset = thickness;
			this.collisionGroup = source.gameObject.layer;
			this.shapeType = shapeType;
			this.shapeIndex = shapeIndex;
			this.rigidbodyIndex = rigidbodyIndex;
			this.materialIndex = materialIndex;
			this.rotationCache = new float[16];
		}

		public void SetSpaceTransform(Transform space){
			bounds.Transform(space.worldToLocalMatrix);
			translation = space.worldToLocalMatrix.MultiplyPoint3x4(translation);
			rotation = Quaternion.Inverse(space.rotation) * rotation;
			scale.Scale(new Vector4(1/space.lossyScale.x,
								    1/space.lossyScale.y,
								    1/space.lossyScale.z,1));
		}
	}

	[Serializable]
	[StructLayout(LayoutKind.Sequential, Pack = 1)]
	public struct CollisionMaterial{
		public float friction;
		public float stickiness;
		public float stickDistance;
		public MaterialCombineMode frictionCombine;
		public MaterialCombineMode stickinessCombine;
	}

	[Serializable]
	[StructLayout(LayoutKind.Sequential, Pack = 1)]
	public struct FluidMaterial{
		public float smoothingRadius;
		public float restDensity;
		public float viscosity;
		public float surfaceTension;
		public float buoyancy;
		public float atmosphericDrag;
		public float atmosphericPressure;
		public float vorticity;
		public float elasticRange;
		public float plasticCreep;
		public float plasticThreshold;
	}

	// In this particular case, size is forced to 32 bytes to ensure 16 byte memory alignment needed by Oni.
	[Serializable]
	[StructLayout(LayoutKind.Sequential, Pack = 1, Size = 32)]
	public struct SphereShape{
		public Vector4 center;	//first 16 bytes
		public float radius;	//next 4 bytes, 12 bytes left unused for padding.

		[MarshalAs(UnmanagedType.I1)]
		public bool is2D;

		public SphereShape(SphereCollider source){
			center = source.center;
			radius = source.radius;
			is2D = false;
		}

		public SphereShape(CircleCollider2D source){
			center = source.offset;
			radius = source.radius;
			is2D = true;
		}
	}

	[Serializable]
	[StructLayout(LayoutKind.Sequential, Pack = 1)]
	public struct BoxShape{
		public Vector4 center;	
		public Vector4 size;

		[MarshalAs(UnmanagedType.I1)]
		public bool is2D;
		
		public BoxShape(BoxCollider source){
			center = source.center;
			size = source.size;
			is2D = false;
        }

		public BoxShape(BoxCollider2D source){
			center = source.offset;
			size = source.size;
			is2D = true;
        }
	}
	
	[Serializable]
	[StructLayout(LayoutKind.Sequential, Pack = 1)]
	public struct CapsuleShape{
		public Vector4 center;	
		public float radius;
		public float height;
		public int direction;
		
		public CapsuleShape(CapsuleCollider source){
			center = source.center;
			radius = source.radius;
			height = source.height;
			direction = source.direction;
		}

		public CapsuleShape(CharacterController source){
			center = source.center;
			radius = source.radius;
			height = source.height;
			direction = 1;
		}
	}
	
	[Serializable]
	[StructLayout(LayoutKind.Sequential, Pack = 1)]
	public struct HeightData{

		TerrainCollider source;
		GCHandle dataHandle;
		
		public HeightData(TerrainCollider source){
			this.dataHandle = new GCHandle();
			this.source = source;
			UpdateHeightData();
        }

		/**
		 * Updates the shared memory region between Obi and Oni where terrain height data resides. 
		 */
		public void UpdateHeightData(){

			float[,] heights = source.terrainData.GetHeights(0,0,source.terrainData.heightmapWidth,source.terrainData.heightmapHeight);
			
			float[] buffer = new float[source.terrainData.heightmapWidth * source.terrainData.heightmapHeight];
			for (int y = 0; y < source.terrainData.heightmapHeight; ++y)
				for (int x = 0; x < source.terrainData.heightmapWidth; ++x)
					buffer[y*source.terrainData.heightmapWidth+x] = heights[y,x];
			
			if (dataHandle.IsAllocated)
				UnpinMemory(dataHandle);

			dataHandle = PinMemory(buffer);
		}
        
        public void UnpinData(){
			UnpinMemory(dataHandle);
        }

		public IntPtr AddrOfHeightData(){
			return dataHandle.AddrOfPinnedObject();
		}        

    }
	
	[Serializable]
	[StructLayout(LayoutKind.Sequential, Pack = 1)]
	public struct HeightmapShape{
		public Vector3 size;
		//pointer to a HeightData object. Done this way because we do not know the array size beforehand, and
		//the struct memory layout must be contiguous.
        public IntPtr data;	 
        public int resolutionU;
		public int resolutionV;
		public float sampleWidth;
		public float sampleHeight;
		
		public HeightmapShape(TerrainCollider source, IntPtr data){
			resolutionU = source.terrainData.heightmapWidth;
			resolutionV = source.terrainData.heightmapHeight;
			sampleWidth = source.terrainData.heightmapScale.x;
			sampleHeight = source.terrainData.heightmapScale.z;
			size = source.terrainData.size;
			this.data = data;
		}
	}

	[Serializable]
	[StructLayout(LayoutKind.Sequential, Pack = 1)]
	public struct TriangleMeshData{
	
		MeshCollider source;
		GCHandle verticesHandle;
		GCHandle normalsHandle;
		GCHandle trianglesHandle;
		
		public TriangleMeshData(MeshCollider source){
			this.verticesHandle = new GCHandle();
			this.trianglesHandle = new GCHandle();
			this.normalsHandle = new GCHandle();
			this.source = source;
			UpdateMeshData();
        }

		/**
		 * Updates the shared memory region between Obi and Oni where triangle mesh data resides. 
		 */
		public void UpdateMeshData(){

			Vector3[] vertices = source.sharedMesh.vertices;
			Vector3[] normals = source.sharedMesh.normals;
			int[] triangles = source.sharedMesh.triangles;
			
			if (verticesHandle.IsAllocated)
				UnpinMemory(verticesHandle);

			if (normalsHandle.IsAllocated)
				UnpinMemory(normalsHandle);

			if (trianglesHandle.IsAllocated)
				UnpinMemory(trianglesHandle);

			verticesHandle = PinMemory(vertices);
			normalsHandle = PinMemory(normals);
			trianglesHandle = PinMemory(triangles);

		}
        
        public void UnpinData(){
			UnpinMemory(verticesHandle);
			UnpinMemory(normalsHandle);
			UnpinMemory(trianglesHandle);
        }

		public IntPtr AddrOfVertexData(){
			return verticesHandle.AddrOfPinnedObject();
		} 

		public IntPtr AddrOfNormalsData(){
			return normalsHandle.AddrOfPinnedObject();
		}  

		public IntPtr AddrOfTriangleData(){
			return trianglesHandle.AddrOfPinnedObject();
		}        

    }

	[Serializable]
	[StructLayout(LayoutKind.Sequential, Pack = 1)]
	public struct TriangleMeshShape{

		public enum MeshColliderType
	    {
	        Solid = 0,
	        ThinOneSided = 1,
	        ThinTwoSided = 2
	    };

		public IntPtr vertexPositions;
        public IntPtr triangleIndices;	
        public int numVertices;
		public int numTriangles;
		public float triangleThickness;
		public MeshColliderType type;
		
		public TriangleMeshShape(MeshCollider source, MeshColliderType type, float thickness,IntPtr vertexPositions, IntPtr triangleIndices){
			this.type = type;
			this.triangleThickness = thickness;
			numVertices = source.sharedMesh.vertexCount;
			numTriangles = source.sharedMesh.triangles.Length/3;
			this.vertexPositions = vertexPositions;
			this.triangleIndices = triangleIndices;
		}
	}

	[Serializable]
	[StructLayout(LayoutKind.Sequential, Pack = 1)]
	public struct EdgeMeshData{
	
		EdgeCollider2D source;
		GCHandle verticesHandle;
		GCHandle edgesHandle;
		
		public EdgeMeshData(EdgeCollider2D source){
			this.verticesHandle = new GCHandle();
			this.edgesHandle = new GCHandle();
			this.source = source;
			UpdateMeshData();
        }

		/**
		 * Updates the shared memory region between Obi and Oni where edge mesh data resides. 
		 */
		public void UpdateMeshData(){

			Vector3[] vertices = new Vector3[source.pointCount];
			int[] edges = new int[source.edgeCount*2];

			for (int i = 0; i < source.pointCount; ++i){
				vertices[i] = source.points[i];
			}

			for (int i = 0; i < source.edgeCount; ++i){
				edges[i*2] = i;
				edges[i*2+1] = i+1;
			}
			
			if (verticesHandle.IsAllocated)
				UnpinMemory(verticesHandle);

			if (edgesHandle.IsAllocated)
				UnpinMemory(edgesHandle);

			verticesHandle = PinMemory(vertices);
			edgesHandle = PinMemory(edges);

		}
        
        public void UnpinData(){
			UnpinMemory(verticesHandle);
			UnpinMemory(edgesHandle);
        }

		public IntPtr AddrOfVertexData(){
			return verticesHandle.AddrOfPinnedObject();
		} 

		public IntPtr AddrOfEdgeData(){
			return edgesHandle.AddrOfPinnedObject();
		}        

    }

	[Serializable]
	[StructLayout(LayoutKind.Sequential, Pack = 1)]
	public struct EdgeMeshShape{

		public IntPtr vertexPositions;
        public IntPtr edgeIndices;	
        public int numVertices;
		public int numEdges;
		public float edgeThickness;
		public bool is2D;
		
		public EdgeMeshShape(EdgeCollider2D source,float thickness,IntPtr vertexPositions, IntPtr edgeIndices){
			this.edgeThickness = thickness;
			numVertices = source.pointCount;
			numEdges = source.edgeCount;
			this.vertexPositions = vertexPositions;
			this.edgeIndices = edgeIndices;
			this.is2D = true;
		}
	}

	[Serializable]
	[StructLayout(LayoutKind.Sequential, Pack = 1)]
	public struct HalfEdge{
		public int index;	
		public int indexInFace;
		public int face;
		public int nextHalfEdge;
		public int pair;
		public int endVertex;

		public HalfEdge(int index){
			this.index = index;	
			indexInFace = -1;
			face = -1;
			nextHalfEdge = -1;
			pair = -1;
			endVertex = -1;
		}
	}

	[Serializable]
	[StructLayout(LayoutKind.Sequential, Pack = 1)]
	public struct Vertex{
		public int index;	
		public int halfEdge;
		public Vector3 position;

		public Vertex(Vector3 position, int index, int halfEdge){
			this.index = index;
			this.halfEdge = halfEdge;
			this.position = position;
		}
	}

	[Serializable]
	[StructLayout(LayoutKind.Sequential, Pack = 1)]
	public struct Face{
		public int index;	
		public int halfEdge;

		public Face(int index){
			this.index = index;
			halfEdge = -1;
		}
	}

	[Serializable]
	[StructLayout(LayoutKind.Sequential, Pack = 1)]
	public struct MeshInformation{
		public float volume;	
		public float area;
		public int borderEdgeCount;

		[MarshalAs(UnmanagedType.I1)]
		public bool closed;
		[MarshalAs(UnmanagedType.I1)]
		public bool nonManifold;
	}

	public static GCHandle PinMemory(object data){
		return GCHandle.Alloc(data, GCHandleType.Pinned);
	}

	public static void UnpinMemory(GCHandle handle){
		handle.Free();
	}

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern IntPtr CreateColliderGroup();
	
	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void DestroyColliderGroup(IntPtr group);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void SetColliders(IntPtr group, Oni.Collider[] colliders, int num, int destOffset);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int RemoveColliders(IntPtr group, int num, int sourceOffset);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int GetColliderCount(IntPtr group);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void SetRigidbodies(IntPtr group, Oni.Rigidbody[] rigidbodies, int num, int destOffset);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int GetRigidbodies(IntPtr group, Oni.Rigidbody[] rigidbodies, int num, int sourceOffset);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int RemoveRigidbodies(IntPtr group, int num, int sourceOffset);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int GetRigidbodyCount(IntPtr group);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int GetShapeCount(IntPtr group, ShapeType shapeType);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void SetSphereShapes(IntPtr group, Oni.SphereShape[] shapes, int num, int destOffset);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int RemoveSphereShapes(IntPtr group, int num, int sourceOffset);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void SetBoxShapes(IntPtr group, Oni.BoxShape[] shapes, int num, int destOffset);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int RemoveBoxShapes(IntPtr group, int num, int sourceOffset);
	
	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void SetCapsuleShapes(IntPtr group, Oni.CapsuleShape[] shapes, int num, int destOffset);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int RemoveCapsuleShapes(IntPtr group, int num, int sourceOffset);
	
	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void SetHeightmapShapes(IntPtr group, Oni.HeightmapShape[] shapes, int num, int destOffset);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int RemoveHeightmapShapes(IntPtr group, int num, int sourceOffset);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void SetTriangleMeshShapes(IntPtr group, Oni.TriangleMeshShape[] shapes, int num, int destOffset);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int RemoveTriangleMeshShapes(IntPtr group, int num, int sourceOffset);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int UpdateTriangleMeshShapes(IntPtr group, int num, int sourceOffset);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void SetEdgeMeshShapes(IntPtr group, Oni.EdgeMeshShape[] shapes, int num, int destOffset);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int RemoveEdgeMeshShapes(IntPtr group, int num, int sourceOffset);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int UpdateEdgeMeshShapes(IntPtr group, int num, int sourceOffset);
        
	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern IntPtr CreateSolver(int maxParticles, int maxNeighbours);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void DestroySolver(IntPtr solver);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void GetBounds(IntPtr solver, ref Vector3 min, ref Vector3 max);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void SetSolverParameters(IntPtr solver, [MarshalAs(UnmanagedType.Struct)] ref SolverParameters parameters);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void GetSolverParameters(IntPtr solver, [MarshalAs(UnmanagedType.Struct)] ref SolverParameters parameters);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int SetActiveParticles(IntPtr solver, int[] active, int num);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void AddSimulationTime(IntPtr solver, float step_dt);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void UpdateSolver(IntPtr solver, float substep_dt);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void ApplyPositionInterpolation(IntPtr solver, float substep_dt);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void UpdateSkeletalAnimation(IntPtr solver);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void GetConstraintsOrder(IntPtr solver, int[] order);
	
	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void SetConstraintsOrder(IntPtr solver, int[] order);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int GetConstraintCount(IntPtr solver, int type);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void GetActiveConstraintIndices(IntPtr solver, int[] indices, int num , int type);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int SetRenderableParticlePositions(IntPtr solver, Vector4[] positions, int num, int destOffset);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int GetRenderableParticlePositions(IntPtr solver, Vector4[] positions,int num, int sourceOffset);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int SetParticlePhases(IntPtr solver, int[] phases, int num, int destOffset);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int SetParticlePositions(IntPtr solver, Vector4[] positions, int num, int destOffset);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int GetParticlePositions(IntPtr solver, Vector4[] positions, int num, int sourceOffset);
	
	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int SetParticleInverseMasses(IntPtr solver, float[] invMasses, int num, int destOffset);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int SetParticleSolidRadii(IntPtr solver, float[] radii, int num, int destOffset);
	
	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int SetParticleVelocities(IntPtr solver, Vector4[] velocities, int num, int destOffset);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int GetParticleVelocities(IntPtr solver, Vector4[] velocities, int num, int sourceOffset);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void AddParticleExternalForces(IntPtr solver, Vector4[] forces, int[] indices, int num);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void AddParticleExternalForce(IntPtr solver, ref Vector4 force, int[] indices, int num);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int GetParticleVorticities(IntPtr solver, Vector4[] vorticities, int num, int sourceOffset);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int SetParticleVorticities(IntPtr solver, Vector4[] vorticities, int num, int destOffset);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int GetParticleNormals(IntPtr solver, Vector4[] normals, int num, int sourceOffset);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int GetParticleDensities(IntPtr solver, float[] densities, int num, int sourceOffset);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int GetDeformableTriangleCount(IntPtr solver);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void SetDeformableTriangles(IntPtr solver, int[] indices, int num, int destOffset);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int RemoveDeformableTriangles(IntPtr solver, int num, int sourceOffset);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void SetConstraintGroupParameters(IntPtr solver, int type, [MarshalAs(UnmanagedType.Struct)] ref ConstraintParameters parameters);
	
	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void GetConstraintGroupParameters(IntPtr solver, int type, [MarshalAs(UnmanagedType.Struct)] ref ConstraintParameters parameters);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void SetColliderGroup(IntPtr solver, IntPtr group);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void SetCollisionMaterials(IntPtr solver, Oni.CollisionMaterial[] materials, int num, int destOffset);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int SetMaterialIndices(IntPtr solver, int[] indices, int num, int destOffset);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int SetRestPositions(IntPtr solver, Vector4[] positions, int num, int destOffset);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void SetFluidMaterials(IntPtr solver, Oni.FluidMaterial[] materials, int num, int destOffset);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int SetFluidMaterialIndices(IntPtr solver, int[] indices, int num, int destOffset);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern IntPtr CreateDeformableMesh(IntPtr solver, 
													 IntPtr halfEdge,
													 IntPtr skinConstraintBatch,
													 [MarshalAs(UnmanagedType.LPArray, SizeConst=16)] float[] worldToLocal, 
													 IntPtr particleIndices, 
													 int vertexCapacity,
													 int vertexCount);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void DestroyDeformableMesh(IntPtr solver, IntPtr mesh);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern bool TearDeformableMeshAtVertex(IntPtr mesh,int vertexIndex,
																	 ref Vector3 planePoint,
																	 ref Vector3 planeNormal,
																	 int[] updated_edges,
																	 ref int num_edges);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void SetDeformableMeshTBNUpdate(IntPtr mesh,NormalsUpdate normalsUpdate, [MarshalAs(UnmanagedType.I1)]bool skinTangents);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void SetDeformableMeshTransform(IntPtr mesh,[MarshalAs(UnmanagedType.LPArray, SizeConst=16)] float[] worldToLocal);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void SetDeformableMeshSkinMap(IntPtr mesh, IntPtr sourceMesh, IntPtr triangleSkinMap);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void SetDeformableMeshParticleIndices(IntPtr mesh, IntPtr particleIndices);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void SetDeformableMeshData(IntPtr mesh,IntPtr triangles,
																IntPtr vertices,
												  	 			IntPtr normals,
																IntPtr tangents,
																IntPtr colors,
																IntPtr uv1,
																IntPtr uv2,
																IntPtr uv3,
																IntPtr uv4);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void SetDeformableMeshAnimationData(IntPtr mesh,float[] bindPoses,BoneWeights[] weights, int numBones);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void SetDeformableMeshBoneTransforms(IntPtr mesh,float[] boneTransforms);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern IntPtr CreateBatch(int type, [MarshalAs(UnmanagedType.I1)]bool cooked);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void DestroyBatch(IntPtr batch);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern IntPtr AddBatch(IntPtr solver, IntPtr batch, [MarshalAs(UnmanagedType.I1)]bool sharesParticles);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void RemoveBatch(IntPtr solver, IntPtr batch);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern bool EnableBatch(IntPtr batch, [MarshalAs(UnmanagedType.I1)]bool enabled);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int GetBatchConstraintCount(IntPtr batch);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int GetBatchConstraintForces(IntPtr batch, float[] forces, int num, int destOffset);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int GetBatchPhaseCount(IntPtr batch);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void GetBatchPhaseSizes(IntPtr batch, int[] phaseSizes);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void SetBatchPhaseSizes(IntPtr batch, int[] phaseSizes, int num);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern bool CookBatch(IntPtr batch);
	
	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int SetActiveConstraints(IntPtr batch, int[] active, int num);

    #if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void SetDistanceConstraints(IntPtr batch, int[] indices,
																   float[] restLengths,
																   Vector2[] stiffnesses,
																   int num);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void GetDistanceConstraints(IntPtr batch, int[] indices,
																   float[] restLengths,
																   Vector2[] stiffnesses);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void SetBendingConstraints(IntPtr batch, int[] indices,
																  float[] restBends,
																  Vector2[] bendingStiffnesses,
																  int num);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void GetBendingConstraints(IntPtr batch, int[] indices,
																  float[] restBends,
																  Vector2[] bendingStiffnesses);


	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void SetSkinConstraints(IntPtr batch, 
												 int[] indices,
												 Vector4[] points,
												 Vector4[] normals,
												 float[] radiiBackstops,
												 float[] stiffnesses,
												 int num);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void GetSkinConstraints(IntPtr batch, 
												 int[] indices,
												 Vector4[] points,
												 Vector4[] normals,
												 float[] radiiBackstops,
												 float[] stiffnesses);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void SetAerodynamicConstraints(IntPtr batch, 
														int[] particleIndices, 
														float[] aerodynamicCoeffs,
														int num);
    
	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void SetVolumeConstraints(IntPtr batch, 
												   int[] triangleIndices,
	                                               int[] firstTriangle,
	                                               int[] numTriangles,
												   float[] restVolumes,
												   Vector2[] pressureStiffnesses,
												   int num);
	
	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void SetTetherConstraints(IntPtr batch, 
												   int[] indices,
												   Vector2[] maxLenghtsScales,
												   float[] stiffnesses,
												   int num);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void GetTetherConstraints(IntPtr batch, int[] indices,
																 Vector2[] maxLenghtsScales,
																 float[] stiffnesses);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void SetPinConstraints(IntPtr batch, 
											    int[] indices,
												Vector4[] pinOffsets,
												float[] stiffnesses,
												int num);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void GetCollisionContacts(IntPtr solver, Contact[] contacts, int n);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void ClearDiffuseParticles(IntPtr solver);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int SetDiffuseParticles(IntPtr solver, Vector4[] positions, int num);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int GetDiffuseParticleVelocities(IntPtr solver, Vector4[] velocities, int num, int sourceOffset);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void SetDiffuseParticleNeighbourCounts(IntPtr solver, IntPtr neighbourCounts);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern IntPtr CreateHalfEdgeMesh();

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void DestroyHalfEdgeMesh(IntPtr mesh);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void SetVertices(IntPtr mesh, IntPtr vertices, int n);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void SetHalfEdges(IntPtr mesh, IntPtr halfedges, int n);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void SetFaces(IntPtr mesh, IntPtr faces, int n);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void SetNormals(IntPtr mesh, IntPtr normals);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void SetTangents(IntPtr mesh, IntPtr tangents);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void SetInverseOrientations(IntPtr mesh, IntPtr orientations);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void SetVisualMap(IntPtr mesh, IntPtr map);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int GetVertexCount(IntPtr mesh);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int GetHalfEdgeCount(IntPtr mesh);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int GetFaceCount(IntPtr mesh);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int GetHalfEdgeMeshInfo(IntPtr mesh, [MarshalAs(UnmanagedType.Struct)] ref MeshInformation meshInfo);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void CalculatePrimitiveCounts(IntPtr mesh, Vector3[] vertices, int[] triangles, int vertexCount, int triangleCount);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void Generate(IntPtr mesh, Vector3[] vertices, int[] triangles, int vertexCount, int triangleCount, ref Vector3 scale);

    #if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int MakePhase(int group, ParticlePhase flags);
	
	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern float BendingConstraintRest(float[] constraintCoordinates);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern IntPtr CreateTriangleSkinMap();

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void DestroyTriangleSkinMap(IntPtr skinmap);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void Bind(IntPtr skinmap, IntPtr sourcemesh, IntPtr targetmesh,  uint[] sourceMasterFlags, uint[] targetSlaveFlags);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int GetSkinnedVertexCount(IntPtr skinmap);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void GetSkinInfo(IntPtr skinmap, 
										 int[] skinIndices, 
										 int[] sourceTriIndices,
										 Vector3[] baryPositions,
										 Vector3[] baryNormals,
										 Vector3[] baryTangents);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void SetSkinInfo(IntPtr skinmap, 
										 int[] skinIndices, 
										 int[] sourceTriIndices,
										 Vector3[] baryPositions,
										 Vector3[] baryNormals,
										 Vector3[] baryTangents,
										 int num);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void WaitForAllTasks();

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void ClearTasks();

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int GetMaxSystemConcurrency();

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void SignalFrameStart();

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern double SignalFrameEnd();

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void EnableProfiler([MarshalAs(UnmanagedType.I1)]bool cooked);

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern int GetProfilingInfoCount();

	#if (UNITY_IPHONE && !UNITY_EDITOR)
		[DllImport ("__Internal")]
	#else
		[DllImport ("libOni")] 
	#endif
	public static extern void GetProfilingInfo([Out] ProfileInfo[] info, int num);
}
