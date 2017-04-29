using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;

namespace Obi{

/**
 * This class allows a mesh (skinned or not) to follow the simulation performed by a different cloth object. The most
 * common use case is having a high-poly mesh mimic the movement of a low-poly cloth simulation. 
 */
[ExecuteInEditMode]
[DisallowMultipleComponent]
[AddComponentMenu("Physics/Obi/Obi Cloth Proxy")]
public class ObiClothProxy : MonoBehaviour {

	public ObiTriangleSkinMap skinMap = null;

	[SerializeField][HideInInspector] private ObiClothBase source;

	[HideInInspector] public Mesh skinnedMesh;
	[HideInInspector] public Mesh sharedMesh;				/**< Original unmodified mesh.*/

	protected int[] meshTriangles;	
	protected Vector3[] meshVertices;
	protected Vector3[] meshNormals;
	protected Vector4[] meshTangents;

	protected IntPtr deformableMesh;
	protected GCHandle meshTrianglesHandle;
	protected GCHandle meshVerticesHandle;
	protected GCHandle meshNormalsHandle;
	protected GCHandle meshTangentsHandle;
	
	private MeshRenderer meshRenderer;
	private MeshFilter meshFilter;
	private SkinnedMeshRenderer skinnedMeshRenderer;
	protected float[] transformData = new float[16];

	public bool SelfSkinning{
		get{return source != null && source.gameObject == gameObject;}
	}

	public ObiClothBase Proxy{
		set{

			if (source != null){
				source.OnDeformableMeshSetup -= Source_OnAddedToSolver;
				source.OnDeformableMeshTeardown -= Source_OnRemovedFromSolver;
			}

				source = value;

			if (source != null){
				source.OnDeformableMeshSetup += Source_OnAddedToSolver;
				source.OnDeformableMeshTeardown += Source_OnRemovedFromSolver;
			}

		}
		get{return source;}
	}

	/*public void OnDrawGizmos(){

		Gizmos.color = Color.blue;
		Matrix4x4 normalMatrix = transform.localToWorldMatrix.inverse.transpose;
		if (meshNormals != null)
		for (int i = 0; i < meshNormals.Length; ++i){
			Gizmos.DrawRay(transform.localToWorldMatrix.MultiplyPoint3x4(meshVertices[i]),normalMatrix.MultiplyVector(meshNormals[i]).normalized*0.02f);
		}

		Gizmos.color = Color.red;
		if (meshTangents != null)
		for (int i = 0; i < meshTangents.Length; ++i){
			Gizmos.DrawRay(transform.localToWorldMatrix.MultiplyPoint3x4(meshVertices[i]),normalMatrix.MultiplyVector(meshTangents[i]).normalized*0.02f);
		}

   	}*/

	void OnEnable(){

		if (!SelfSkinning){
			if (skinnedMeshRenderer == null)
				InitializeWithRegularMesh();
			else{
				Debug.LogWarning("Cloth proxies do not work with SkinnedMeshRenderers yet.");
			}
		}

		if (source != null){
			source.OnDeformableMeshSetup += Source_OnAddedToSolver;
			source.OnDeformableMeshTeardown += Source_OnRemovedFromSolver;
		}
	}

	void OnDisable(){

		if (!SelfSkinning){
			if (meshFilter != null)
				meshFilter.mesh = sharedMesh;
			//if (skinnedMeshRenderer != null)
			//	skinnedMeshRenderer.sharedMesh = sharedMesh;
	
			GameObject.DestroyImmediate(skinnedMesh);
		}

		if (source != null){
			source.OnDeformableMeshSetup -= Source_OnAddedToSolver;
			source.OnDeformableMeshTeardown -= Source_OnRemovedFromSolver;
		}
	}

	private void UpdateTransformData()
	{
		if (source == null) return;

		Matrix4x4 w2lTransform = transform.worldToLocalMatrix;
		if (SelfSkinning)
			w2lTransform = source.ActorWorldToLocalMatrix;

		Matrix4x4 s2lTransform;
		if (source.Solver.simulateInLocalSpace)
			s2lTransform = w2lTransform * source.Solver.transform.localToWorldMatrix;
		else 
			s2lTransform = w2lTransform;
		
		for (int i = 0; i < 16; ++i)
			transformData[i] = s2lTransform[i];
	}

	void Source_OnAddedToSolver (object sender, EventArgs e)
	{
		UpdateTransformData();

		// In case source and target are the same object:
		if (SelfSkinning){

			deformableMesh = source.DeformableMesh;

		// If source and target are different objects, create a new deformable mesh for the target:
		}else{

			deformableMesh = Oni.CreateDeformableMesh(source.Solver.OniSolver,
													  IntPtr.Zero,
													  IntPtr.Zero,
													  transformData,
													  IntPtr.Zero,
												      sharedMesh.vertexCount,
													  sharedMesh.vertexCount);
	
			GetMeshDataArrays(skinnedMesh);
		}

		Oni.SetDeformableMeshSkinMap(deformableMesh,source.DeformableMesh,skinMap.TriangleSkinMap);

		source.Solver.OnFrameEnd += Source_Solver_OnFrameEnd;
	}

	void Source_OnRemovedFromSolver (object sender, EventArgs e)
	{
		source.Solver.OnFrameEnd -= Source_Solver_OnFrameEnd;

		if (!SelfSkinning){
			Oni.DestroyDeformableMesh(source.Solver.OniSolver,deformableMesh);
			Oni.UnpinMemory(meshVerticesHandle);
			Oni.UnpinMemory(meshNormalsHandle);
			Oni.UnpinMemory(meshTangentsHandle);
		}
	}

	void Source_Solver_OnFrameEnd (object sender, EventArgs e)
	{
		if (!SelfSkinning && skinnedMesh != null){
			skinnedMesh.vertices = meshVertices;
			skinnedMesh.normals = meshNormals;
			skinnedMesh.tangents = meshTangents;
			skinnedMesh.RecalculateBounds();
		}
	}

	public virtual void GetMeshDataArrays(Mesh mesh){

		if (mesh != null)
		{
			Oni.UnpinMemory(meshTrianglesHandle);
			Oni.UnpinMemory(meshVerticesHandle);
			Oni.UnpinMemory(meshNormalsHandle);
			Oni.UnpinMemory(meshTangentsHandle);

			meshTriangles = mesh.triangles;
			meshVertices = mesh.vertices;
			meshNormals = mesh.normals;
			meshTangents = mesh.tangents;

			meshTrianglesHandle = Oni.PinMemory(meshTriangles);
			meshVerticesHandle = Oni.PinMemory(meshVertices);
			meshNormalsHandle = Oni.PinMemory(meshNormals);
			meshTangentsHandle = Oni.PinMemory(meshTangents);

			Oni.SetDeformableMeshData(deformableMesh,meshTrianglesHandle.AddrOfPinnedObject(),
													 meshVerticesHandle.AddrOfPinnedObject(),
													 meshNormalsHandle.AddrOfPinnedObject(),
													 meshTangentsHandle.AddrOfPinnedObject(),
													 IntPtr.Zero,IntPtr.Zero,IntPtr.Zero,IntPtr.Zero,IntPtr.Zero);
		}
	}

	private void InitializeWithRegularMesh(){
		
		meshFilter = GetComponent<MeshFilter>();
		meshRenderer = GetComponent<MeshRenderer>();
		
		if (meshFilter == null || meshRenderer == null)
			return;
		
		// Store the shared mesh if it hasn't been stored previously.
		if (sharedMesh != null)
			meshFilter.mesh = sharedMesh;
		else
			sharedMesh = meshFilter.sharedMesh;
		
		// Make a deep copy of the original shared mesh.
		skinnedMesh = Mesh.Instantiate(meshFilter.sharedMesh) as Mesh;
		skinnedMesh.MarkDynamic(); 
		
		// Use the freshly created mesh copy as the renderer mesh and the half-edge input mesh, if it has been already analyzed.
		meshFilter.mesh = skinnedMesh;

	}

	public void Update(){

		UpdateTransformData();

		if (deformableMesh != IntPtr.Zero)
			Oni.SetDeformableMeshTransform(deformableMesh,transformData);
	}
	
}
}
