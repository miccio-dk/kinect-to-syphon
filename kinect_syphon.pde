// Daniel Shiffman
// Kinect Point Cloud example

// https://github.com/shiffman/OpenKinect-for-Processing
// http://shiffman.net/p5/kinect/

import org.openkinect.freenect.*;
import org.openkinect.processing.*;
import codeanticode.syphon.*;

SyphonServer server;


// Kinect Library object
Kinect kinect;
PVector v_old_x = new PVector(0, 0);
PVector v_old_y = new PVector(0, 0);

// Angle for rotation
float a = 0;

int skipX = 2;
int skipY = 2;
float zoom_factor = 1000;
float min_thrsh = 42.0;
float max_thrsh = 636.0; // 970
float max_dist = 14 * skipX;

// We'll use a lookup table so that we don't have to repeat the math over and over
float[] depthLookUp = new float[2048];
int[][] mat;
int n_rows;
int n_cols;

boolean limit = true;
PVector zero = new PVector(0., 0., 0.);

void setup() {
  // Rendering in P3D
  //fullScreen(P3D, 0);
  size(800, 600, P3D);
  kinect = new Kinect(this);
  kinect.initDepth();

  // Lookup table for all possible depth values (0 - 2047)
  for (int i = 0; i < depthLookUp.length; i++) {
    depthLookUp[i] = rawDepthToMeters(i);
  }
  
  n_rows = kinect.height / skipY;
  n_cols = kinect.width / skipX;
  mat = new int[n_cols][n_rows];
  
  background(0);
  server = new SyphonServer(this, "Processing");
}

void draw() {
  background(0);

  // Get the raw depth as array of integers
  int[] depth = kinect.getRawDepth();
  
  // Translate and rotate
  translate(width/2, height/2, -zoom_factor/2.);
  rotateY(mouseX * 6.28 / width - 3.14);
  rotateX(mouseY * 6.28 / height - 3.14);
  a += 0.01f;
  
  // Nested for loop that initializes x and y pixels and, for those less than the
  // maximum threshold and at every skiping point, the offset is caculated to map
  // them on a plane instead of just a line
  for (int y = 0, j=0; y < kinect.height; y += skipY, j++) {
    for (int x = 0, i=0; x < kinect.width; x += skipX, i++) {
      int offset = x + y*kinect.width;
      int rawDepth = depth[offset];
      
      mat[i][j] = rawDepth; // v.z;
    }
  }
  
  for (int j=1; j < n_rows; j++) {
    for (int i=1; i < n_cols; i++) {
      int rawDepth = mat[i][j];
      if (!limit || rawDepth > min_thrsh  && rawDepth < max_thrsh) { 
        PVector v = depthToWorld(i*skipX, j*skipY, rawDepth);
        v.mult(zoom_factor);
        PVector v_old_x = depthToWorld((i-1)*skipX, j*skipY, mat[i-1][j]);
        v_old_x.mult(zoom_factor);
        PVector v_old_y = depthToWorld(i*skipX, (j-1)*skipY, mat[i][j-1]);
        v_old_y.mult(zoom_factor);
               
        if(v.dist(zero) < 5. || v_old_x.dist(zero) < 5. || v_old_y.dist(zero) < 5.)
          continue;
        
        fill(255);
        stroke(255);
        strokeWeight(0.3);

        if(false && i==1 && j==1) {
          text(" " + v, 10, 10);
          text(" " + v_old_x, 10, 30);
          text(" " + v_old_y, 10, 50);
          //println(" " + v + " " + v_old_x + " " + v_old_y);
        }

        pushMatrix();
        
        if(!limit || v.dist(v_old_x) < max_dist) {
          line(v.x, v.y, zoom_factor - v.z, v_old_x.x, v_old_x.y, zoom_factor - v_old_x.z);
        }
        if(!limit || v.dist(v_old_y) < max_dist) {
          line(v.x, v.y, zoom_factor - v.z, v_old_y.x, v_old_y.y, zoom_factor - v_old_y.z);
        }
        popMatrix();
      }
    }
  }
  
  server.sendScreen();
}


// These functions come from: http://graphics.stanford.edu/~mdfisher/Kinect.html
float rawDepthToMeters(int depthValue) {
  if (depthValue < 2047) {
    return (float)(1.0 / ((double)(depthValue) * -0.0030711016 + 3.3309495161));
  }
  return 0.0f;
}

// Only needed to make sense of the ouput depth values from the kinect
PVector depthToWorld(int x, int y, int depthValue) {

  final double fx_d = 1.0 / 5.9421434211923247e+02;
  final double fy_d = 1.0 / 5.9104053696870778e+02;
  final double cx_d = 3.3930780975300314e+02;
  final double cy_d = 2.4273913761751615e+02;

// Drawing the result vector to give each point its three-dimensional space
  PVector result = new PVector();
  double depth =  depthLookUp[depthValue];//rawDepthToMeters(depthValue);
  result.x = (float)((x - cx_d) * depth * fx_d);
  result.y = (float)((y - cy_d) * depth * fy_d);
  result.z = (float)(depth);
  return result;
}
