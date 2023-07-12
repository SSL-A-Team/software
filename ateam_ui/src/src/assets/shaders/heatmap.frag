#define GLSLIFY 1

//The resolution and coordinates of the current pixel
// uniform vec2 iResolution;
// uniform vec2 vTextureSize;
// varying vec2 vTextureCoord;

// uniform sampler2D heatmap

precision mediump float;

varying vec2 vTextureCoord;//The coordinates of the current pixel
uniform sampler2D uSampler;//The image data

void main(void) {
   gl_FragColor = texture2D(uSampler, vTextureCoord);
}