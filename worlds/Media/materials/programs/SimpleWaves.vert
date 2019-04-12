//***************************************************    
// * Title: UUV Simulator   
// * Author: The UUV Simulator Authors  
// * Date: 2016      
// * Availability: https://uuvsimulator.github.io/
//***************************************************

// Input parameters
uniform vec3 eyePos;
uniform float rescale;
uniform vec2 bumpScale;
uniform vec2 bumpSpeed;
uniform float time;
uniform float frequency;
uniform float amplitude;
uniform float steepness;

// Output variables
varying mat3 rotMatrix;
varying vec3 eyeVec;
varying vec2 bumpCoord;

// Compute linear combination of Gerstner waves as described in
// GPU Gems, chapter 01: "Effective Water Simulation from Physical Models"
// http://http.developer.nvidia.com/GPUGems/gpugems_ch01.html

// Information regarding a single wave
struct WaveParameters {
    float w;   // frequency
    float a;   // amplitude
    float phi; // phase constant of speed
    vec2 d;    // horizontal direction of wave
	float q;   // steepness for Gerstner wave (q=0: rolling sine waves)
};

void main(void)
{
    // Use combination of three waves. Values here are chosen rather arbitrarily.
    // Other parameters might lead to better-looking waves.

    #define N_WAVES 3
    WaveParameters waves[N_WAVES];
    waves[0] = WaveParameters(frequency, 0.6*amplitude, 0.5, vec2(-1, 0), steepness);
    waves[1] = WaveParameters(3.2*frequency, 0.4*amplitude, 1.7, vec2(-0.7, 0.7), 1.5*steepness);
	waves[2] = WaveParameters(1.8*frequency, 0.3*amplitude, 1.0, vec2(0.7, 0.7), 0.8*steepness);

    vec4 P = gl_Vertex;

    // Iteratively compute binormal, tangent, and normal vectors:
    vec3 B = vec3(1.0, 0.0, 0.0);
    vec3 T = vec3(0.0, 1.0, 0.0);
    vec3 N = vec3(0.0, 0.0, 1.0);

	// Wave synthesis using linear combination of Gerstner waves
	for(int i = 0; i < N_WAVES; ++i)
	{
	    // Evaluate wave equation:
        float angle = dot(waves[i].d, P.xy)*waves[i].w + time*waves[i].phi;
		float c = cos(angle);
		float s = sin(angle);
        float q = waves[i].q;

        // Displacement of point due to wave (Eq. 9)
		P.x += q*waves[i].a*c*waves[i].d.x;
		P.y += q*waves[i].a*c*waves[i].d.y;
		P.z += waves[i].a*s;

        // Modify normals due to wave displacement (Eq. 10-12)
		float wa = waves[i].a*waves[i].w;
		float qwas = q*wa*s;
		float wac = wa*c;
		float dx = waves[i].d.x;
		float dy = waves[i].d.y;
		float dxy = dx*dy;

		B += vec3(-qwas*dx*dx, -qwas*dxy, wac*dx);
		T += vec3(-qwas*dxy, -qwas*dy*dy, wac*dy);
		N += vec3(-dx*wac, -dy*wac, -qwas);
	}

    // Compute (Surf2World * Rescale) matrix
    B = normalize(B)*rescale;
    T = normalize(T)*rescale;
    N = normalize(N);
	rotMatrix = mat3(B, T, N);

	gl_Position = gl_ModelViewProjectionMatrix*P;

	// Compute texture coordinates for bump map
	bumpCoord = gl_MultiTexCoord0.xy*bumpScale + time*bumpSpeed;

	eyeVec = P.xyz - eyePos; // eye position in vertex space
}
