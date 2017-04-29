using UnityEngine;
using System;
using System.Threading;
using System.Collections;
using System.Collections.Generic;

namespace Obi{

[ExecuteInEditMode]
[RequireComponent(typeof(ObiActor))]
public class ObiParticleRenderer : MonoBehaviour
{
	public bool render = true;
	public Color particleColor = Color.white; 
	public float radiusScale = 1;

	private ObiActor actor;
	private List<Mesh> meshes = new List<Mesh>();
	private Material material;

	// Geometry buffers:
	private Vector3[] vertices = new Vector3[0];
	private Vector3[] normals = new Vector3[0];
	private Vector2[] uv = new Vector2[0];
	private Vector2[] uv2 = new Vector2[0];
	private Color[] colors = new Color[0];
	int[] triangles = new int[0];

	private Vector2[] particleUVs = new Vector2[4]{
		Vector2.one,
		Vector2.up,
		Vector2.zero,
		Vector2.right
	};

	private Vector3[] particleOffsets = new Vector3[4]{
		new Vector3(1,1,0),
		new Vector3(-1,1,0),
		new Vector3(-1,-1,0),
		new Vector3(1,-1,0)
	};

	public IEnumerable<Mesh> ParticleMeshes{
		get { return meshes; }
	}

	public Material ParticleMaterial{
		get { return material; }
	}

	public void Awake(){
		actor = GetComponent<ObiActor>();
	}

	public void OnEnable(){

		material = GameObject.Instantiate(Resources.Load("ObiMaterials/Particle",typeof(Material)) as Material);
		material.hideFlags = HideFlags.HideAndDontSave;

		if (actor != null && actor.Solver != null)
		{
			actor.Solver.RequireRenderablePositions();
			actor.Solver.OnFrameEnd += Actor_solver_OnFrameEnd;
		} 
	}

	public void OnDisable(){

		if (actor != null && actor.Solver != null)
		{
			actor.Solver.RelinquishRenderablePositions();
			actor.Solver.OnFrameEnd -= Actor_solver_OnFrameEnd;
		}

		ClearMeshes();

		GameObject.DestroyImmediate(material);
	}

	void Actor_solver_OnFrameEnd (object sender, EventArgs e)
	{
		if (actor == null || !actor.InSolver || !actor.isActiveAndEnabled)
			return;

		ObiSolver solver = actor.Solver;

		// Update particle renderer values:
		Vector3[] particlePositions = new Vector3[actor.particleIndices.Length];
		Vector2[] particleInfo = new Vector2[actor.particleIndices.Length];
		Color[] particleColors = new Color[actor.particleIndices.Length];

		int activeParticleCount = 0;

		for (int i = 0; i < actor.particleIndices.Length; i++){
			if (actor.Solver.activeParticles.Contains(i))
			{
				particlePositions[activeParticleCount] = solver.renderablePositions[actor.particleIndices[i]];
				particleInfo[activeParticleCount] = new Vector2(0,actor.solidRadii[i]*radiusScale);

				if (actor.colors != null && i < actor.colors.Length)
					particleColors[activeParticleCount] = actor.colors[i];
				else 
					particleColors[activeParticleCount] = Color.white;

				activeParticleCount++;
			}
		}

		material.SetColor("_Color",particleColor);
		
		SetParticles(activeParticleCount,particlePositions,particleInfo,particleColors);
	
		if (render)
			DrawParticles();
	}

	private void DrawParticles(){

		// Send the meshes to be drawn:
		foreach(Mesh mesh in meshes)
			Graphics.DrawMesh(mesh, Vector3.zero, Quaternion.identity, material, 0);

	}

	private void Resize(int particleCount){

		Array.Resize(ref vertices,particleCount*4);
		Array.Resize(ref normals,particleCount*4);
		Array.Resize(ref uv,particleCount*4);
		Array.Resize(ref uv2,particleCount*4);
		Array.Resize(ref colors,particleCount*4);
		Array.Resize(ref triangles,particleCount*6);

		ObiUtils.ArrayFill<Vector2>(uv,particleUVs);
		ObiUtils.ArrayFill<Vector3>(normals,particleOffsets);

	}

	private void Apply(Mesh mesh){
		mesh.Clear();
		mesh.vertices = vertices;
		mesh.normals = normals;
		mesh.uv = uv;
		mesh.uv2 = uv2;
		mesh.colors = colors;
		mesh.triangles = triangles;
		mesh.RecalculateBounds();
	}

	private void ClearMeshes(){
		foreach(Mesh mesh in meshes)
			GameObject.DestroyImmediate(mesh);
		meshes.Clear();
	}

	public void SetParticles(int activeParticleCount,Vector3[] positions, Vector2[] info, Color[] colors){

		if (positions == null || info == null){
			ClearMeshes();
			return;
		}

		// Figure out how many meshes we are going to need to draw all particles:
		int particlesPerDrawcall = Constants.maxVertsPerMesh/4;
		int drawcallCount = activeParticleCount / particlesPerDrawcall + 1;
		particlesPerDrawcall = Mathf.Min(particlesPerDrawcall,activeParticleCount);

		Resize(particlesPerDrawcall);

		// If the amount of meshes we need to draw the particles has changed:
		if (drawcallCount != meshes.Count){

			// Re-generate meshes:
			ClearMeshes();
			for (int i = 0; i < drawcallCount; i++){
				Mesh mesh = new Mesh();
				mesh.name = "Particle imposters";
				mesh.hideFlags = HideFlags.HideAndDontSave;
				mesh.MarkDynamic();
				meshes.Add(mesh);
			}

		}

		//Convert particle data to mesh geometry:
		for (int i = 0; i < drawcallCount; i++){

			for(int j = i * particlesPerDrawcall, index = 0; 
				j < activeParticleCount && j < (i+1) * particlesPerDrawcall;
			    ++j,++index)
			{
				SetParticle(index,positions[j],
	                        info[j],
							colors[j]);
			}

			Apply(meshes[i]);
		}

	}

	private void SetParticle(int i, Vector3 position, Vector2 info, Color color){
		
		int i4 = i*4;
		int i41 = i4+1;
		int i42 = i4+2;
		int i43 = i4+3;
		int i6 = i*6;
		
		vertices[i4] = position;
		vertices[i41] = position;
		vertices[i42] = position;
		vertices[i43] = position;

		uv2[i4] = info;
		uv2[i41] = info;
		uv2[i42] = info;
		uv2[i43] = info;
		
		colors[i4] = color;
		colors[i41] = color;
		colors[i42] = color;
        colors[i43] = color;

		triangles[i6] = i42;
		triangles[i6+1] = i41;
		triangles[i6+2] = i4;
		triangles[i6+3] = i43;
        triangles[i6+4] = i42;
        triangles[i6+5] = i4;
    }

}
}

