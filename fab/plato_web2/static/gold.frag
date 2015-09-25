#extension GL_OES_standard_derivatives : enable

varying vec3 vPosition;

vec3 getNormal() {
  // Differentiate the position vector
  vec3 dPositiondx = dFdx(vPosition);
  vec3 dPositiondy = dFdy(vPosition);

  // The normal is the cross product of the differentials
  return normalize(cross(dPositiondx, dPositiondy));
}

float fresnel(float f0, vec3 n, vec3 l){
  return f0 + (1.0-f0) * pow(1.0- dot(n, l), 5.0);
}

float diffuseEnergyRatio(float f0, vec3 n, vec3 l){
  return 1.0 - fresnel(f0, n, l);
}



//Beckmann
float distribution(vec3 n, vec3 h, float roughness){
  float m_Sq= roughness * roughness;
  float NdotH_Sq= max(dot(n, h), 0.0);
  NdotH_Sq= NdotH_Sq * NdotH_Sq;
  return exp( (NdotH_Sq - 1.0)/(m_Sq*NdotH_Sq) )/ (3.14159265 * m_Sq * NdotH_Sq * NdotH_Sq) ;
}


// Schlick
float geometry(vec3 n, vec3 h, vec3 v, vec3 l, float roughness){
  float NdotH= dot(n, h);
  float NdotL= dot(n, l);
  float NdotV= dot(n, v);
  float VdotH= dot(v, h);
  float NdotL_clamped= max(NdotL, 0.0);
  float NdotV_clamped= max(NdotV, 0.0);
  return min( min( 2.0 * NdotH * NdotV_clamped / VdotH, 2.0 * NdotH * NdotL_clamped / VdotH), 1.0);
}




precision highp float;

//varying vec3    v_position;
//varying vec3    v_normal;
//uniform float	u_fresnel0;
//uniform float	u_roughness;
//uniform vec3	u_diffuseColor;
//uniform vec3	u_lightColor;
//uniform vec3	u_lightDir;     // in view space

vec3 getBrdfTerm( float u_roughness,
                  vec3 u_diffuseColor,
                  vec3 u_lightColor,
                  vec3 u_lightDir,
                  float refractiveIndex) {
 float u_fresnel0 = (1.0 - refractiveIndex)/(1.0 + refractiveIndex);
  u_fresnel0 = u_fresnel0 * u_fresnel0;

  vec3 normal =  getNormal();
  vec3 view   = -normalize(vPosition - cameraPosition); //vec3(0, 1.9, 3.7));
  vec3 halfVec=  normalize(u_lightDir + view);
  float NdotL= dot(normal, u_lightDir);
  float NdotV= dot(normal, view);
  float NdotL_clamped= max(NdotL, 0.0);
  float NdotV_clamped= max(NdotV, 0.0);

 float brdf_spec= fresnel(u_fresnel0, halfVec, u_lightDir) * geometry(normal, halfVec, view, u_lightDir, u_roughness) * distribution(normal, halfVec, u_roughness) / (4.0 * NdotL_clamped * NdotV_clamped);
  vec3 color_spec= NdotL_clamped * brdf_spec * u_lightColor;
  vec3 color_diff= NdotL_clamped * diffuseEnergyRatio(u_fresnel0, normal, u_lightDir) * u_diffuseColor * u_lightColor;
  return color_diff + color_spec;
}

vec4 plastic_color(vec4 color, vec4 dark, vec4 light) {
  vec3 spotLight = vec3(-10, 5, 10);
  vec4 spot_color = vec4(0.8, 0.4, 0.6, 1.0);

  vec3 uLight = vec3(10, 10, 10);
  //vec4 color = vec4(1.0, 0, 0, 1.0);
  //vec4 dark = vec4(0, 0, 0, 1.0);
  //vec4 light = vec4(1.0, 1.0, 1.0, 1.0);
  vec3 normal = getNormal();

  // Mix in diffuse light
  float diffuse = dot(normalize(uLight - vPosition), normal);
  diffuse = max(0.0, diffuse);
  color = mix(dark, color, 0.1 + 0.9 * diffuse);

  // Diffuse from spotlight
  float spot = dot(normalize(spotLight - vPosition), normal);
  spot = max(0.0, spot);
  color = mix(color, spot_color, 0.2 * spot);

  // Mix in specular light
  vec3 halfVector = normalize(normalize(cameraPosition - vPosition) + normalize(uLight - vPosition));
  float specular = dot(normal, halfVector);
  specular = max(0.0, specular);
  specular = pow(specular, 50.0);
  color = mix(color, light, 0.5 * specular);

  halfVector = normalize(normalize(cameraPosition - vPosition) + normalize(spotLight - vPosition));
  specular = dot(normal, halfVector);
  specular = max(0.0, specular);
  specular = pow(specular, 50.0);
  color = mix(color, spot_color * 1.1, 0.3 * specular);

  return color;
}


void main() {
  vec3 brdf = getBrdfTerm(0.15, //24,
                          vec3(1.0/3.14, 1.0/3.14, 0.1/3.14),
                          vec3(1.0 * 3.14, 0.8 * 3.14, 0.8 * 3.14),
                          normalize(vec3(-1.0, 1.0, 1.0)),
                          2.0);

  vec4 base = vec4(0.32, 0.13, 0, 1);
  vec4 base_lighter = vec4(0.57, 0.27, 0, 1);
  vec4 orange = vec4(1.0, 0.5, 0, 1);
  vec4 brdf_vec4 = vec4(brdf, 1.0);
  vec4 plast = plastic_color(base_lighter, base, orange);


  vec4 composite =  brdf_vec4 + plast; //base; //mix(brdf_vec4, red_plast, 0.8);
  composite = clamp(composite, plast, vec4(1.0, 1.0, 1.0, 1.0));

  gl_FragColor = composite;
}
