Shader "Obi/EditorLines" 
{
	SubShader 
	{ 
		Pass 
		{
		    Blend SrcAlpha OneMinusSrcAlpha 
			Cull Off Fog { Mode Off }  
		    BindChannels 
			{
		      Bind "vertex", vertex Bind "color", color 
			}
		} 
	} 
}