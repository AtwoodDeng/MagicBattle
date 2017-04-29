using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Obi{

	[RequireComponent(typeof(MeshCollider))]
    public class ObiClothPicker : MonoBehaviour {

		public class ParticlePickEventArgs : EventArgs{

			public int particleIndex;
			public Vector3 mouseDelta;
	
			public ParticlePickEventArgs(int particleIndex, Vector3 mouseDelta){
				this.particleIndex = particleIndex;
				this.mouseDelta = mouseDelta;
			}
		}
    
		public event EventHandler<ParticlePickEventArgs> OnParticlePicked;
		public event EventHandler<ParticlePickEventArgs> OnParticleDragged;
		public event EventHandler<ParticlePickEventArgs> OnParticleReleased;

		private MeshCollider collider;
		private ObiClothBase cloth;
		private Mesh currentCollisionMesh;

		private Vector3 lastMousePos = Vector3.zero;
		private int pickedParticleIndex = -1;
		private float pickedParticleDepth = 0;

		public ObiClothBase Cloth{
			get{return cloth;}
		}

		void Awake(){
			cloth = GetComponent<ObiClothBase>();
			collider = GetComponent<MeshCollider>();
			collider.enabled = false;
			lastMousePos = Input.mousePosition;
		}

		void OnEnable(){
			if (cloth != null)
				cloth.Solver.OnFrameBegin += Cloth_Solver_OnFrameBegin;
		}

		void OnDisable(){
			if (cloth != null)
				cloth.Solver.OnFrameBegin -= Cloth_Solver_OnFrameBegin;
		}

		void Cloth_Solver_OnFrameBegin (object sender, EventArgs e)
		{
			// Click:
            if (Input.GetMouseButtonDown(0)){

				GameObject.Destroy(currentCollisionMesh);
				currentCollisionMesh = GameObject.Instantiate(cloth.clothMesh);
				collider.sharedMesh = currentCollisionMesh;

				Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);

				RaycastHit hitInfo;
				if (collider.Raycast(ray,out hitInfo,100)){

					int[] tris = currentCollisionMesh.triangles;
					Vector3[] vertices = currentCollisionMesh.vertices;

					// find closest vertex in the triangle we just hit:
					int closestVertex = -1;
					float minDistance = float.MaxValue;

					for (int i = 0; i < 3; ++i){
						int vertex = tris[hitInfo.triangleIndex*3+i];
						float distance = (vertices[vertex] - hitInfo.point).sqrMagnitude;
						if (distance < minDistance){
							minDistance = distance;
							closestVertex = vertex;
						}
					}
			
					// get particle index:
					if (closestVertex >= 0 && closestVertex < cloth.topology.visualMap.Length){

						pickedParticleIndex = cloth.topology.visualMap[closestVertex];
						pickedParticleDepth = Mathf.Abs((cloth.transform.TransformPoint(vertices[closestVertex]) - Camera.main.transform.position).z);

						if (OnParticlePicked != null)
							OnParticlePicked(this,new ParticlePickEventArgs(pickedParticleIndex, Vector3.zero));
					}
				}

			}else if (pickedParticleIndex >= 0){

				// Drag:
				Vector3 mouseDelta = Input.mousePosition - lastMousePos;
				if (mouseDelta.magnitude > 0.01f && OnParticleDragged != null){

					Vector3 worldMouseDelta = Camera.main.ScreenToWorldPoint (new Vector3 (Input.mousePosition.x, Input.mousePosition.y, pickedParticleDepth)) -
											  Camera.main.ScreenToWorldPoint (new Vector3 (lastMousePos.x, lastMousePos.y, pickedParticleDepth));

					OnParticleDragged(this,new ParticlePickEventArgs(pickedParticleIndex,worldMouseDelta));
				}


				// Release:				
				if (Input.GetMouseButtonUp(0)){
					
					if (OnParticleReleased != null)
						OnParticleReleased(this,new ParticlePickEventArgs(pickedParticleIndex,Vector3.zero));
					pickedParticleIndex = -1;
	
				}
			}
		
			lastMousePos = Input.mousePosition;
		}
    }
}
