using UnityEngine;
using System.Collections;
using System;

namespace Obi{

/**
 * Holds information about the physical properties of the substance emitted by an emitter.
 */
public class ObiEmitterMaterial : ScriptableObject
{

	public class MaterialChangeEventArgs : EventArgs{

		public MaterialChanges changes;

		public MaterialChangeEventArgs(MaterialChanges changes){
			this.changes = changes;
		}
	}

	[Flags]
	public enum MaterialChanges{
		PER_MATERIAL_DATA = 0,
		PER_PARTICLE_DATA = 1 << 0
	}

	public enum MaterialDimensions{
		Mode3D = 3,
		Mode2D = 2
	}

	public bool isFluid = true;				/**< do the emitter particles generate density constraints?*/

	[SerializeField][HideInInspector] MaterialDimensions mode = MaterialDimensions.Mode3D;
	[SerializeField][HideInInspector] private float resolution = 1;
	[SerializeField][HideInInspector] private float smoothing = 1.5f;
	[SerializeField][HideInInspector] private float particleMass = 1;
	[SerializeField][HideInInspector] private float restDensity = 1000;		/**< rest density of the fluid particles.*/
	[SerializeField][HideInInspector] private float restRadius = 0.1f;
	[SerializeField][HideInInspector] private float smoothingRadius = 0.15f;

	// fluid parameters:
	public float randomRadius = 0.0f;		/**< A random amount between 0 and randomRadius gets added to each particle if the material is set to non-fluid.*/
	public float viscosity = 0.05f;			/**< viscosity of the fluid particles.*/
	public float surfaceTension = 0.1f;	/**< surface tension of the fluid particles.*/

	// gas parameters:
	public float buoyancy = -1.0f; 						/**< how dense is this material with respect to air?*/
	public float atmosphericDrag = 0;					/**< amount of drag applied by the surrounding air to particles near the surface of the material.*/
	public float atmosphericPressure = 0;				/**< amount of pressure applied by the surrounding air particles.*/
	public float vorticity = 0.0f;						/**< amount of baroclinic vorticity injected.*/
	
	// elastoplastic parameters:
	//public float elasticRange; 		/** radius around a particle in which distance constraints are created.*/
	//public float plasticCreep;		/**< rate at which a deformed plastic material regains its shape*/
	//public float plasticThreshold;	/**< amount of stretching stress that a elastic material must undergo to become plastic.*/

	public MaterialDimensions Mode{
		get{return mode;}
		set{
			mode = value;
			restRadius = 1f / (10 * Mathf.Pow(resolution,1/(float)mode));
			smoothingRadius = restRadius * smoothing;

			if (isFluid)
				particleMass = restDensity * Mathf.Pow(restRadius,(int)mode);
			else
				restDensity = particleMass / Mathf.Pow(restRadius,(int)mode);
		}
	}

	public float Resolution{
		get{return resolution;}
		set{
			resolution = Mathf.Max(0.001f,value);
			restRadius = 1f / (10 * Mathf.Pow(resolution,1/(float)mode));
			particleMass = restDensity * Mathf.Pow(restRadius,(int)mode);
			smoothingRadius = restRadius * smoothing;
		}
	}

	public float Smoothing{
		get{return smoothing;}
		set{
			smoothing = Mathf.Max(1,value);
			smoothingRadius = restRadius * smoothing;
		}
	}

	public float ParticleMass{
		get{return particleMass;}
		set{
			particleMass = value;
			restDensity = particleMass / Mathf.Pow(restRadius,(int)mode);
		}
	}

	public float RestDensity{
		get{return restDensity;}
		set{
			restDensity = value;
			particleMass = restDensity * Mathf.Pow(restRadius,(int)mode);
		}
	}

	public float RestRadius{
		get{return restRadius;}
		set{
			restRadius = Mathf.Max(0.000001f,value);
			particleMass = restDensity * Mathf.Pow(restRadius,(int)mode);
			smoothing = smoothingRadius / restRadius;
			resolution = (float)Mathf.Pow(1.0f / (restRadius * 10),(int)mode);
		}
	}

	public float SmoothingRadius{
		get{return smoothingRadius;}
		set{
			smoothingRadius = Mathf.Max(restRadius,value);
			smoothing = smoothingRadius / restRadius;
		}
	}

	private EventHandler<MaterialChangeEventArgs> onChangesMade;
	public event EventHandler<MaterialChangeEventArgs> OnChangesMade {
	
	    add {
	        onChangesMade -= value;
	        onChangesMade += value;
	    }
	    remove {
	        onChangesMade -= value;
	    }
	}

	public void CommitChanges(MaterialChanges changes){
		if (onChangesMade != null)
				onChangesMade(this,new MaterialChangeEventArgs(changes));
	}

	public void OnValidate(){

		viscosity = Mathf.Max(0,viscosity);
		atmosphericDrag = Mathf.Max(0,atmosphericDrag);

	}

	public Oni.FluidMaterial GetEquivalentOniMaterial()
	{
		Oni.FluidMaterial material = new Oni.FluidMaterial();
		material.smoothingRadius = smoothingRadius;
		material.restDensity = restDensity;
		material.viscosity = viscosity;
		material.surfaceTension = surfaceTension;
		material.buoyancy = buoyancy;
		material.atmosphericDrag = atmosphericDrag;
		material.atmosphericPressure = atmosphericPressure;
		material.vorticity = vorticity;
		return material;
	}
}
}

