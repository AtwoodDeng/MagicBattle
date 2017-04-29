using System;
using UnityEngine;

namespace Obi
{
	[RequireComponent(typeof(ObiClothPicker))]
	public class ObiClothDragger : MonoBehaviour
	{
		private ObiClothPicker picker;
		private float originalMass = 0;


		void OnEnable()
		{
			picker = GetComponent<ObiClothPicker>();
			picker.OnParticlePicked += Picker_OnParticlePicked;
			picker.OnParticleDragged += Picker_OnParticleDragged;
			picker.OnParticleReleased += Picker_OnParticleReleased;
		}

		void OnDisable()
		{
			picker.OnParticlePicked -= Picker_OnParticlePicked;
			picker.OnParticleDragged -= Picker_OnParticleDragged;
			picker.OnParticleReleased -= Picker_OnParticleReleased;
		}

		void Picker_OnParticleReleased (object sender, ObiClothPicker.ParticlePickEventArgs e)
		{
			Oni.SetParticleInverseMasses(picker.Cloth.Solver.OniSolver,new float[]{originalMass},1,picker.Cloth.particleIndices[e.particleIndex]);
			picker.Cloth.Solver.RelinquishRenderablePositions();
		}

		void Picker_OnParticleDragged (object sender, ObiClothPicker.ParticlePickEventArgs e)
		{
			if (originalMass > 0){
				Vector4 newPosition = picker.Cloth.Solver.renderablePositions[picker.Cloth.particleIndices[e.particleIndex]] +
									  new Vector4(e.mouseDelta[0],e.mouseDelta[1],e.mouseDelta[2],0);
	
				Oni.SetParticlePositions(picker.Cloth.Solver.OniSolver,new Vector4[]{newPosition},1,picker.Cloth.particleIndices[e.particleIndex]);
			}
		}

		void Picker_OnParticlePicked (object sender, ObiClothPicker.ParticlePickEventArgs e)
		{
			picker.Cloth.Solver.RequireRenderablePositions();
			originalMass = picker.Cloth.invMasses[e.particleIndex];
			Oni.SetParticleVelocities(picker.Cloth.Solver.OniSolver,new Vector4[]{Vector4.zero},1,picker.Cloth.particleIndices[e.particleIndex]);
			Oni.SetParticleInverseMasses(picker.Cloth.Solver.OniSolver,new float[]{0},1,picker.Cloth.particleIndices[e.particleIndex]);
		}
	}
}

