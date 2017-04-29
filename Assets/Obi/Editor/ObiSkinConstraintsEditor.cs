using UnityEditor;
using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;

namespace Obi{
	
	/**
	 * Custom inspector for ObiSkinConstraints component. 
	 */
	
	[CustomEditor(typeof(ObiSkinConstraints)), CanEditMultipleObjects] 
	public class ObiSkinConstraintsEditor : Editor
	{
		
		ObiSkinConstraints constraints;
		
		public void OnEnable(){
			constraints = (ObiSkinConstraints)target;
		}
		
		public override void OnInspectorGUI() {
			
			serializedObject.UpdateIfDirtyOrScript();
			
			Editor.DrawPropertiesExcluding(serializedObject,"m_Script");
			
			// Apply changes to the serializedProperty
			if (GUI.changed){
				
				serializedObject.ApplyModifiedProperties();
				
				constraints.PushDataToSolver();
				
			}
			
		}

		public void OnSceneGUI(){

			if (Event.current.type != EventType.Repaint || !ObiParticleActorEditor.editMode) return;
			
			// Get the particle actor editor to retrieve selected particles:
			ObiParticleActorEditor[] editors = (ObiParticleActorEditor[])Resources.FindObjectsOfTypeAll(typeof(ObiParticleActorEditor));

			// If there's any particle actor editor active, we can show pin constraints:
			if (editors.Length > 0 && (editors[0].currentProperty == ObiClothEditor.ClothParticleProperty.SkinRadius ||
									   editors[0].currentProperty == ObiClothEditor.ClothParticleProperty.SkinBackstopRadius ||
									   editors[0].currentProperty == ObiClothEditor.ClothParticleProperty.SkinBackstop) )
 			{

				Material mat = ObiEditorUtils.GetRequiredEditorResource("Obi/EditorLines.mat") as Material;
			
				if (mat.SetPass(0) && constraints.GetBatches().Count > 0) {

					GL.PushMatrix();
					GL.Begin(GL.LINES);

					ObiSkinConstraintBatch batch = ((ObiSkinConstraintBatch) constraints.GetBatches()[0]);

					Matrix4x4 s2wTransform = constraints.Actor.Solver.transform.localToWorldMatrix;

					// get up to date constraint data:
					batch.PullDataFromSolver(constraints);
				
					foreach (int i in batch.ActiveConstraints){
	
						int particleIndex = batch.skinIndices[i];

						bool[] stat = ObiParticleActorEditor.selectionStatus;
						
						if (particleIndex >= 0 && particleIndex < ObiParticleActorEditor.selectionStatus.Length && 
							ObiParticleActorEditor.selectionStatus[particleIndex]){
	
							float radius = batch.skinRadiiBackstop[i*3];
							float collisionRadius = batch.skinRadiiBackstop[i*3+1];
							float backstop = batch.skinRadiiBackstop[i*3+2];
							Vector3 point = batch.GetSkinPosition(i);
							Vector3 normal = batch.GetSkinNormal(i);

							if (!constraints.InSolver){

								point = constraints.transform.TransformPoint(point);
								normal = constraints.transform.TransformDirection(normal);

							}else if (constraints.Actor.Solver.simulateInLocalSpace){

								point = s2wTransform.MultiplyPoint3x4(point);
								normal = s2wTransform.MultiplyVector(normal);
		
							}

							// Detailed visual feedback only if few particles are selected, to avoid clutter.
							if (ObiParticleActorEditor.SelectedParticleCount < 5){

								Handles.color = new Color(0,0,1,0.2f);
								Handles.SphereCap(0,point,Quaternion.identity,radius*2);
								Handles.color = new Color(1,0,0,0.2f);
								Handles.SphereCap(0,point - normal*(collisionRadius + backstop),Quaternion.identity,collisionRadius*2);

							// If more than 4 particles are selected, use lines.
							}else{
								GL.Color(Color.red);
								GL.Vertex(point);
								GL.Color(Color.red);
	        					GL.Vertex(point - normal * backstop);
	
								GL.Color(Color.blue);
								GL.Vertex(point - normal * backstop);
								GL.Color(Color.blue);
	        					GL.Vertex(point + normal * radius);
							}
							
						}
					}

					GL.End();
					GL.PopMatrix();

				}

			}
			
		}
		
	}
}

