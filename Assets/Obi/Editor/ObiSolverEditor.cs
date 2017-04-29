using UnityEditor;
using UnityEngine;
using UnityEditorInternal;
using System;
using System.Collections;
using System.Collections.Generic;

namespace Obi{
	
/**
 * Custom inspector for ObiSolver components.
 * Allows particle selection and constraint edition. 
 * 
 * Selection:
 * 
 * - To select a particle, left-click on it. 
 * - You can select multiple particles by holding shift while clicking.
 * - To deselect all particles, click anywhere on the object except a particle.
 * 
 * Constraints:
 * 
 * - To edit particle constraints, select the particles you wish to edit.
 * - Constraints affecting any of the selected particles will appear in the inspector.
 * - To add a new pin constraint to the selected particle(s), click on "Add Pin Constraint".
 * 
 */
	[CustomEditor(typeof(ObiSolver)), CanEditMultipleObjects] 
	public class ObiSolverEditor : Editor
	{
		
		[MenuItem("Component/Physics/Obi/Obi Solver",false,0)]
		static void AddObiSolver()
		{
			foreach(Transform t in Selection.transforms)
				Undo.AddComponent<ObiSolver>(t.gameObject);
		}

		[MenuItem("GameObject/3D Object/Obi/Obi Solver",false,0)]
		static void CreateObiSolver()
		{
			GameObject c = new GameObject("Obi Solver");
			Undo.RegisterCreatedObjectUndo(c,"Create Obi Solver");
			c.AddComponent<ObiSolver>();
		}
		
		[MenuItem("GameObject/3D Object/Obi/Obi Solver (with collider group)",false,1)]
		static void CreateObiSolverWithColliderGroup()
		{
			GameObject c = new GameObject("Obi Solver");
			Undo.RegisterCreatedObjectUndo(c,"Create Obi Solver");
			ObiSolver solver = c.AddComponent<ObiSolver>();
			ObiColliderGroup group = c.AddComponent<ObiColliderGroup>();
			solver.colliderGroup = group;
		}

		ObiSolver solver;
		private ReorderableList constraintOrderList;
		
		public void OnEnable(){
			solver = (ObiSolver)target;

			constraintOrderList = new ReorderableList(serializedObject, 
                									  serializedObject.FindProperty("constraintsOrder"), 
                									  true, true, false, false);

			constraintOrderList.drawHeaderCallback = (Rect rect) => {  
				EditorGUI.LabelField(rect, "Constraint enforce order");
			};

			constraintOrderList.drawElementCallback = (Rect rect, int index, bool isActive, bool isFocused) => {
			    int element = constraintOrderList.serializedProperty.GetArrayElementAtIndex(index).intValue;
				EditorGUI.LabelField( new Rect(rect.x, rect.y+2, rect.width, EditorGUIUtility.singleLineHeight),((Oni.ConstraintType)element).ToString());
			};

			constraintOrderList.onReorderCallback = (ReorderableList l) => { 
				solver.UpdateParameters();
			};
		}
		
		public override void OnInspectorGUI() {
			
			serializedObject.UpdateIfDirtyOrScript();
			int oldMaxParticles = solver.maxParticles;

			if (solver.allocatedParticles != null)
				EditorGUILayout.HelpBox("Used particles:"+ solver.allocatedParticles.Count,MessageType.Info);			

			Editor.DrawPropertiesExcluding(serializedObject,"m_Script");
		
			constraintOrderList.DoLayoutList();

            // Apply changes to the serializedProperty
            if (GUI.changed){

                serializedObject.ApplyModifiedProperties();

				if (oldMaxParticles != solver.maxParticles){
					solver.Initialize();
				}

				solver.UpdateParameters();

            }
            
        }
        
		[DrawGizmo (GizmoType.InSelectionHierarchy | GizmoType.Selected)]
		static void DrawGizmoForSolver(ObiSolver solver, GizmoType gizmoType) {
	
			if ((gizmoType & GizmoType.InSelectionHierarchy) != 0) {
	
				Gizmos.color = new Color(1,1,1,0.5f);
				Bounds wsBounds = solver.simulateInLocalSpace ? solver.Bounds.Transform(solver.transform.localToWorldMatrix) : solver.Bounds;
				Gizmos.DrawWireCube(wsBounds.center, wsBounds.size);
			}
	
		}

	}
}


