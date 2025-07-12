"use client"

import React from "react"
import { extend, useFrame } from "@react-three/fiber"
import * as THREE from "three"
import { shaderMaterial } from "@react-three/drei"

// Custom metallic shader
const MetalMaterial = shaderMaterial(
  {
    time: 0,
    color: new THREE.Color(0.5, 0.5, 0.75),
    roughness: 0.5,
    metalness: 0.8,
    scratches: 0.5,
    envMap: null,
  },
  // Vertex Shader
  `
    varying vec3 vPosition;
    varying vec3 vNormal;
    varying vec2 vUv;
    varying vec3 vViewPosition;
    
    void main() {
      vUv = uv;
      vPosition = position;
      vNormal = normalize(normalMatrix * normal);
      
      vec4 mvPosition = modelViewMatrix * vec4(position, 1.0);
      vViewPosition = -mvPosition.xyz;
      
      gl_Position = projectionMatrix * mvPosition;
    }
  `,
  // Fragment Shader
  `
    uniform float time;
    uniform vec3 color;
    uniform float roughness;
    uniform float metalness;
    uniform float scratches;
    uniform samplerCube envMap;
    
    varying vec3 vPosition;
    varying vec3 vNormal;
    varying vec2 vUv;
    varying vec3 vViewPosition;
    
    // Simplex noise function
    vec3 mod289(vec3 x) { return x - floor(x * (1.0 / 289.0)) * 289.0; }
    vec4 mod289(vec4 x) { return x - floor(x * (1.0 / 289.0)) * 289.0; }
    vec4 permute(vec4 x) { return mod289(((x*34.0)+1.0)*x); }
    vec4 taylorInvSqrt(vec4 r) { return 1.79284291400159 - 0.85373472095314 * r; }
    
    float snoise(vec3 v) {
      const vec2 C = vec2(1.0/6.0, 1.0/3.0);
      const vec4 D = vec4(0.0, 0.5, 1.0, 2.0);
      
      // First corner
      vec3 i  = floor(v + dot(v, C.yyy));
      vec3 x0 = v - i + dot(i, C.xxx);
      
      // Other corners
      vec3 g = step(x0.yzx, x0.xyz);
      vec3 l = 1.0 - g;
      vec3 i1 = min(g.xyz, l.zxy);
      vec3 i2 = max(g.xyz, l.zxy);
      
      vec3 x1 = x0 - i1 + C.xxx;
      vec3 x2 = x0 - i2 + C.yyy;
      vec3 x3 = x0 - D.yyy;
      
      // Permutations
      i = mod289(i);
      vec4 p = permute(permute(permute(
                i.z + vec4(0.0, i1.z, i2.z, 1.0))
              + i.y + vec4(0.0, i1.y, i2.y, 1.0))
              + i.x + vec4(0.0, i1.x, i2.x, 1.0));
              
      // Gradients: 7x7 points over a square, mapped onto an octahedron.
      float n_ = 0.142857142857;
      vec3 ns = n_ * D.wyz - D.xzx;
      
      vec4 j = p - 49.0 * floor(p * ns.z * ns.z);
      
      vec4 x_ = floor(j * ns.z);
      vec4 y_ = floor(j - 7.0 * x_);
      
      vec4 x = x_ *ns.x + ns.yyyy;
      vec4 y = y_ *ns.x + ns.yyyy;
      vec4 h = 1.0 - abs(x) - abs(y);
      
      vec4 b0 = vec4(x.xy, y.xy);
      vec4 b1 = vec4(x.zw, y.zw);
      
      vec4 s0 = floor(b0)*2.0 + 1.0;
      vec4 s1 = floor(b1)*2.0 + 1.0;
      vec4 sh = -step(h, vec4(0.0));
      
      vec4 a0 = b0.xzyw + s0.xzyw*sh.xxyy;
      vec4 a1 = b1.xzyw + s1.xzyw*sh.zzww;
      
      vec3 p0 = vec3(a0.xy, h.x);
      vec3 p1 = vec3(a0.zw, h.y);
      vec3 p2 = vec3(a1.xy, h.z);
      vec3 p3 = vec3(a1.zw, h.w);
      
      // Normalise gradients
      vec4 norm = taylorInvSqrt(vec4(dot(p0,p0), dot(p1,p1), dot(p2, p2), dot(p3,p3)));
      p0 *= norm.x;
      p1 *= norm.y;
      p2 *= norm.z;
      p3 *= norm.w;
      
      // Mix final noise value
      vec4 m = max(0.6 - vec4(dot(x0,x0), dot(x1,x1), dot(x2,x2), dot(x3,x3)), 0.0);
      m = m * m;
      return 42.0 * dot(m*m, vec4(dot(p0,x0), dot(p1,x1), dot(p2,x2), dot(p3,x3)));
    }
    
    // PBR shading
    vec3 getNormal() {
      // Add subtle surface irregularities
      float scale = 10.0;
      float scratchStrength = scratches * 0.02;
      
      float noise1 = snoise(vec3(vUv * scale, time * 0.1)) * scratchStrength;
      float noise2 = snoise(vec3(vUv * scale * 2.0 + 100.0, time * 0.05)) * scratchStrength;
      
      // Create micro scratches along primary directions
      float scratchX = step(0.7, fract(vUv.x * 30.0 + noise1)) * scratchStrength;
      float scratchY = step(0.7, fract(vUv.y * 40.0 + noise2)) * scratchStrength;
      
      // Combine noise and scratches with normal
      vec3 perturbedNormal = normalize(vNormal + vec3(noise1 + scratchX, noise2 + scratchY, 0.0));
      return perturbedNormal;
    }
    
    void main() {
      vec3 normal = getNormal();
      vec3 viewDir = normalize(vViewPosition);
      
      // Fresnel (edge lighting)
      float fresnel = 0.3 + 0.7 * pow(1.0 - max(0.0, dot(normal, viewDir)), 5.0);
      
      // Fake environment reflections
      vec3 reflectDir = reflect(-viewDir, normal);
      vec3 baseColor = color;
      
      // Diffuse component
      float diffuse = max(0.0, dot(normal, normalize(vec3(1.0, 1.0, 1.0))));
      
      // Specular component
      vec3 halfDir = normalize(viewDir + normalize(vec3(1.0, 1.0, 1.0)));
      float specular = pow(max(0.0, dot(normal, halfDir)), 30.0 * (1.0 - roughness));
      
      // Combine components with metalness
      vec3 finalColor = mix(
        baseColor * (0.3 + 0.7 * diffuse), // Non-metallic
        baseColor * (0.5 + 0.5 * diffuse + specular), // Metallic
        metalness
      );
      
      // Add fresnel effect
      finalColor = mix(finalColor, vec3(1.0), fresnel * metalness * 0.5);
      
      // Add noise variations for realism
      float grain = snoise(vec3(vUv * 1000.0, time)) * 0.03;
      finalColor += grain;
      
      gl_FragColor = vec4(finalColor, 1.0);
    }
  `,
)

// Extend Three.js with our custom shader
extend({ MetalMaterial })

export function MetalShaderMaterial({ color, roughness = 0.5, metalness = 0.8, scratches = 0.5 }) {
  const ref = React.useRef()

  useFrame((state) => {
    if (ref.current) {
      ref.current.time = state.clock.elapsedTime
    }
  })

  return (
    <metalMaterial
      ref={ref}
      color={color}
      roughness={roughness}
      metalness={metalness}
      scratches={scratches}
      attach="material"
    />
  )
}

