#ifndef OBIPARTICLES_INCLUDED
#define OBIPARTICLES_INCLUDED

#include "UnityCG.cginc"
#include "AutoLight.cginc"

float4x4 _Camera_to_World;
float4x4 _World_to_Camera;
float4x4 _InvProj;

float3 _FarCorner;

float _ThicknessCutoff;

sampler2D _MainTex;
sampler2D _Thickness;
sampler2D _Normals;
sampler2D _CameraDepthTexture;

struct fout {
 	half4 color : COLOR;
	float depth : DEPTH;
};

float3 BillboardSphereNormals(float2 texcoords)
{
	float3 n;
	n.xy = texcoords*2.0-1.0;
	float r2 = dot(n.xy, n.xy);
	clip (1 - r2);   // clip pixels outside circle
	n.z = sqrt(1.0 - r2);
	return n;
}

float BillboardSphereThickness(float2 texcoords)
{
	float2 n = texcoords*2.0-1.0;
	float r2 = dot(n.xy, n.xy);
	clip (1 - r2);   // clip pixels outside circle
	return sqrt(1.0 - r2)*2.0f*exp(-r2*2.0f);
}

half3 SampleSphereAmbient(float3 eyeNormal)
{
	#if UNITY_SHOULD_SAMPLE_SH
		half3 worldNormal = mul(transpose((float3x3)UNITY_MATRIX_V),eyeNormal);  
		#if (SHADER_TARGET < 30)
			return ShadeSH9(half4(worldNormal, 1.0));
		#else
			// Optimization: L2 per-vertex, L0..L1 per-pixel
			return ShadeSH3Order(half4(worldNormal, 1.0));
		#endif
	#else
		return UNITY_LIGHTMODEL_AMBIENT;
	#endif
}

float3 EyePosFromDepth(float2 uv,float eyeDepth){

	float3 ray = (half3(-0.5f,-0.5f,0) + half3(uv.xy,-1)) * _FarCorner;
	return ray * eyeDepth / _FarCorner.z;

}

float SetupEyeSpaceFragment(in float2 uv, out float3 eyePos, out float3 eyeNormal)
{
	float eyeZ = tex2D(_MainTex, uv).r;
	float thickness = tex2D(_Thickness,uv).a;

	if (thickness * 10 < _ThicknessCutoff)
		discard;

	// reconstruct eye space position/direction from frustum corner and camera depth:
	eyePos = EyePosFromDepth(uv,eyeZ);

	// get normal from texture: 
	eyeNormal = (tex2D(_Normals,uv)-0.5) * 2;

	return thickness;
}

void GetWorldSpaceFragment(in float3 eyePos, in float3 eyeNormal, 
						   out float3 worldPos, out float3 worldNormal, out float3 worldView)
{
	// Get world space position, normal and view direction:
	worldPos 	= mul(_Camera_to_World,half4(eyePos,1)).xyz;
	worldNormal = mul((float3x3)_Camera_to_World,eyeNormal);
	worldView   = normalize(UnityWorldSpaceViewDir(worldPos.xyz));
}

void OutputFragmentDepth(in float3 eyePos, inout fout fo)
{
	float4 clipPos = mul(unity_CameraProjection,float4(eyePos,1));
	fo.depth = clipPos.z/clipPos.w;

	#if SHADER_API_OPENGL || SHADER_API_GLCORE	
		fo.depth = 0.5*fo.depth + 0.5;
	#endif
}

#endif
