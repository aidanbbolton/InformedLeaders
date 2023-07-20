

#include "WGP.h"

#define N 200
#define PARAM 6

#define BOARDER 64
#define B_DIM 18

int th=15;          //  Azimuth of view angle
int ph=5;          //  Elevation of view angle
double zh=0; 	   //  time

double asp=1;      //  Aspect Ratio
double dim=32;	   //Size of the box
double b_scaled = (BOARDER/2)*0.75;

//Lighting values
int stop = 1;
int colors=0;
int axes=1;
int mode=1;  	   //  Switch between modes


// double pitch=0;
// double roll=0;
// double yaw=0;
double r_inner = 2;
double r_outer = 8;
double w = 1.3;

double speed = 0.06;
double rotate_speed = 0.05;
double info_thresh = 25;
double v = 0;

double Accuracy_vec[3] = {0,0,0}; 


double boid_locations[N][PARAM];
double boarders[BOARDER][BOARDER][B_DIM];

void setup()
{
	for(int i=0; i<N; i++)
	{
		for(int j=0; j<PARAM; j++)
		{
			
			boid_locations[i][j] = 0;
			if(j<3) boid_locations[i][j] = (i-N/2)/dim;
			
		}
	}
	
	for(int i=0; i<BOARDER; i++)
	{
		for(int j=0; j<BOARDER; j++)
		{
			boarders[i][j][0] = i - b_scaled;
			boarders[i][j][1] = j - b_scaled;
			boarders[i][j][2] = -b_scaled;
			
			boarders[i][j][3] = -b_scaled;
			boarders[i][j][4] = j - b_scaled;
			boarders[i][j][5] = i - b_scaled;
			
			boarders[i][j][6] = i - b_scaled;
			boarders[i][j][7] = j - b_scaled;
			boarders[i][j][8] = b_scaled;
			
			boarders[i][j][9] = b_scaled;
			boarders[i][j][10] = j - b_scaled;
			boarders[i][j][11] = i - b_scaled;
			
			boarders[i][j][12] = i - b_scaled;
			boarders[i][j][13] = -b_scaled;
			boarders[i][j][14] = j - b_scaled;
			
			boarders[i][j][15] = i - b_scaled;
			boarders[i][j][16] = b_scaled;
			boarders[i][j][17] = j - b_scaled;
		}
	}
}



/*
	Stores normal to vector in 3
*/
void Normal(double* P, double* Q, double* R, double* cross) {
				   
	double u[3] = {0,0,0};
	double v[3] = {0,0,0};
	
	for(int i=0;i<3;i++) {
		u[i] = P[i] - Q[i];
		v[i] = R[i] - P[i];
	}
	
	cross[0] = (u[1] * v[2]) - (u[2] *v[1]);
	cross[1] = (u[2] * v[0]) - (u[0] *v[2]);
	cross[2] = (u[0] * v[1]) - (u[1] *v[0]);
				   
}

// Theta = cos^-1 (u . v) (||u|| . ||v||)
// dot prod = u . v = u1*v1 + u2*v2
double dot(double* u, double* v) {

	double sum = 0;
	for(int i=0; i<3; i++)
	{
		sum += u[i]*v[i];
	}
	return sum;

}

double angle_from_vectors2d(double* u, double* v)
{

	double num = u[0]*v[0] + u[1]*v[1];
	double den = sqrt(pow(u[0],2) + pow(u[1],2)) * sqrt(pow(v[0],2) + pow(v[1],2));
	
	return revCos(num/den);
	
}

void rotate_vector(double* Norm, double x, double y, double z)
{
	double Nx = Norm[0];
	double Ny = Norm[1];
	double Nz = Norm[2];
	
	double R00 = Cos(y)*Cos(z);
	double R10 = Cos(y)*Sin(z);
	double R20 = -1*Sin(y);
	
	double R01 = Sin(x)*Sin(y)*Cos(z) - Cos(x)*Sin(z);
	double R11 = Sin(x)*Sin(y)*Sin(z) + Cos(x)*Cos(z);
	double R21 = Sin(x)*Cos(y);
	
	double R02 = Cos(x)*Sin(y)*Cos(z) + Sin(x)*Sin(z);
	double R12 = Cos(x)*Sin(y)*Sin(z) - Sin(x)*Cos(z);
	double R22 = Cos(x)*Cos(y);
	
	Norm[0] = Nx*R00 + Ny*R01 + Nz*R02;
	Norm[1] = Nx*R10 + Ny*R11 + Nz*R12;
	Norm[2] = Nx*R20 + Ny*R21 + Nz*R22;
}

void Vertex(double th,double ph)
{
	double x = Sin(th)*Cos(ph);
	double y = Cos(th)*Cos(ph);
	double z =         Sin(ph);

	glNormal3d(x,y,z);
	glVertex3d(x,y,z);
}

void draw_boarders() {

	int b = BOARDER-1;
	glPushMatrix();
	glColor3f(0,1,1);
	int d=90;
	
	for(int rh=0; rh<360;rh+=d)
	{
		glRotated(rh,0,1,0);
		
		glBegin(GL_LINES);
		glVertex3d(-b_scaled,-b_scaled,-b_scaled);
		glVertex3d( b_scaled,-b_scaled,-b_scaled);
	
		glVertex3d( b_scaled,-b_scaled,-b_scaled);
		glVertex3d( b_scaled, b_scaled,-b_scaled);
	
		glVertex3d( b_scaled, b_scaled,-b_scaled);
		glVertex3d(-b_scaled, b_scaled,-b_scaled);
	
		glVertex3d(-b_scaled, b_scaled,-b_scaled);
		glVertex3d(-b_scaled,-b_scaled,-b_scaled);
		glEnd();
		
	}
	
	

	glPopMatrix();


}

void dumb_interact(double* Normal,double x,double y,double z, int i){
// Find neighbors

	int inner_exists = 0;
	int outer_exists = 0;
	
	double inner_vec[3] = {0,0,0};
	double outer_vec[3] = {0,0,0};
	
	
	for(int j=0;j<N;j++)
	{
		double jx     = boid_locations[j][0];
		double jy     = boid_locations[j][1];
		double jz     = boid_locations[j][2];
		double jpitch = boid_locations[j][3];
		double jroll  = boid_locations[j][4];
		double jyaw   = boid_locations[j][5];
		
		double jcross[3] = {0,1,0};
		rotate_vector(jcross,jpitch,jroll,jyaw);
	
		double dist = sqrt(pow(jx-x,2) + pow(jy-y,2) + pow(jz-z,2));

		double len = sqrt(pow(jx-x,2) + pow(jy-y,2) + pow(jz-z,2));
// 		if(i!=j)printf("%d,%d,%.1f\n",i,j,dist);
		if(dist < r_inner && i != j)
		{
			inner_exists = 1;
			inner_vec[0] += (jx - x) / len;
			inner_vec[1] += (jy - y) / len;
			inner_vec[2] += (jz - z) / len;
		} else if(dist < r_outer && i != j) {
		
			outer_exists = 1;
			double jcrossLen = sqrt(pow(jcross[0],2) + pow(jcross[1],2) + pow(jcross[2],2));
			outer_vec[0] += ((jx - x) / len) + (jcross[0]/jcrossLen);
			outer_vec[1] += ((jy - y) / len) + (jcross[1]/jcrossLen);
			outer_vec[2] += ((jz - z) / len) + (jcross[2]/jcrossLen);
		}
	}
	
// 	if((int)zh %30 == 0) {
// 	
// 	
// 		for(int j=0;j<BOARDER;j++){
// 			for(int k=0; k<BOARDER;k++) {
// 				for(int l=0; l<18;l+=3) {
// 			
// 					double bx = boarders[j][k][l];
// 					double by = boarders[j][k][l+1];
// 					double bz = boarders[j][k][l+2];
// 				
// 					double dist = sqrt(pow(bx-x,2) + pow(by-y,2) + pow(bz-z,2));
// 					double len = sqrt(pow(bx-x,2) + pow(by-y,2) + pow(bz-z,2));
// 				
// 					if(dist < r_inner) {
// 						inner_exists = 1;
// 						rotate_speed=1;
// 						inner_vec[0] += (bx - x) / len;
// 						inner_vec[1] += (by - y) / len;
// 						inner_vec[2] += (bz - z) / len;
// 					}
// 				}
// 			}
// 		}
// 	}
	double innerLen = sqrt(pow(inner_vec[0],2) + pow(inner_vec[1],2) + pow(inner_vec[2],2));
	double outerLen = sqrt(pow(outer_vec[0],2) + pow(outer_vec[1],2) + pow(outer_vec[2],2));
	
	
	for(int j=0;j<3;j++) {
	
		inner_vec[j] = inner_vec[j]/innerLen;
		outer_vec[j] = outer_vec[j]/outerLen;
	}
	
	if(inner_exists)
	{
		Normal[0] -= inner_vec[0];
		Normal[1] -= inner_vec[1];
		Normal[2] -= inner_vec[2];
	} else if(outer_exists) {
		Normal[0] += outer_vec[0];
		Normal[1] += outer_vec[1];
		Normal[2] += outer_vec[2];
	}

}

void informed_interact(double* Normal,double x,double y,double z, int i){
// Find neighbors

	int inner_exists = 0;
	int outer_exists = 0;
	
	double inner_vec[3] = {0,0,0};
	double outer_vec[3] = {0,0,0};
	
	double target_vec[3] = {1,1,1};
	
	
	for(int j=0;j<N;j++)
	{
		double jx     = boid_locations[j][0];
		double jy     = boid_locations[j][1];
		double jz     = boid_locations[j][2];
		double jpitch = boid_locations[j][3];
		double jroll  = boid_locations[j][4];
		double jyaw   = boid_locations[j][5];
		
		double jcross[3] = {0,1,0};
		rotate_vector(jcross,jpitch,jroll,jyaw);
	
		double dist = sqrt(pow(jx-x,2) + pow(jy-y,2) + pow(jz-z,2));

		double len = sqrt(pow(jx-x,2) + pow(jy-y,2) + pow(jz-z,2));
// 		if(i!=j)printf("%d,%d,%.1f\n",i,j,dist);
		if(dist < r_inner && i != j)
		{
			inner_exists = 1;
			inner_vec[0] += (jx - x) / len;
			inner_vec[1] += (jy - y) / len;
			inner_vec[2] += (jz - z) / len;
		} else if(dist < r_outer && i != j) {
		
			outer_exists = 1;
			double jcrossLen = sqrt(pow(jcross[0],2) + pow(jcross[1],2) + pow(jcross[2],2));
			outer_vec[0] += ((jx - x) / len) + (jcross[0]/jcrossLen);
			outer_vec[1] += ((jy - y) / len) + (jcross[1]/jcrossLen);
			outer_vec[2] += ((jz - z) / len) + (jcross[2]/jcrossLen);
		}
	}
// 	if((int)zh%30==0){
// 	
// 		for(int j=0;j<BOARDER;j++){
// 			for(int k=0; k<BOARDER;k++) {
// 				for(int l=0; l<18;l+=3) {
// 			
// 					double bx = boarders[j][k][l];
// 					double by = boarders[j][k][l+1];
// 					double bz = boarders[j][k][l+2];
// 				
// 					double dist = sqrt(pow(bx-x,2) + pow(by-y,2) + pow(bz-z,2));
// 					double len = sqrt(pow(bx-x,2) + pow(by-y,2) + pow(bz-z,2));
// 				
// 					if(dist < r_inner) {
// 						inner_exists = 1;
// 						rotate_speed=1;
// 						inner_vec[0] += (bx - x) / len;
// 						inner_vec[1] += (by - y) / len;
// 						inner_vec[2] += (bz - z) / len;
// 					}
// 				}
// 			}
// 		}
// 	}
	double innerLen = sqrt(pow(inner_vec[0],2) + pow(inner_vec[1],2) + pow(inner_vec[2],2));
	double outerLen = sqrt(pow(outer_vec[0],2) + pow(outer_vec[1],2) + pow(outer_vec[2],2));
	double targetLen = sqrt(pow(target_vec[0]*w,2) + pow(target_vec[1]*w,2) + pow(target_vec[2]*w,2));
	
	
	//Normalize before target
	for(int j=0;j<3;j++) {
	
		inner_vec[j] = inner_vec[j]/innerLen;
		outer_vec[j] = outer_vec[j]/outerLen;
	}
	
	if(inner_exists)
	{
		Normal[0] -= inner_vec[0];
		Normal[1] -= inner_vec[1];
		Normal[2] -= inner_vec[2];
	} else if(outer_exists) {
		Normal[0] += outer_vec[0];
		Normal[1] += outer_vec[1];
		Normal[2] += outer_vec[2];
	}
	Normal[0] += (target_vec[0]*w)/targetLen;
	Normal[1] += (target_vec[1]*w)/targetLen;
	Normal[2] += (target_vec[2]*w)/targetLen;	
	
	double NormLen = sqrt(pow(Normal[0],2) + pow(Normal[1],2) + pow(Normal[2],2));
	for(int j=0;j<3;j++) {
	
		Normal[j] = Normal[j]/NormLen;
		
	}

}

void boid(int i,int informed)
{

	glPushMatrix();
	
	double x   = boid_locations[i][0];
	double y   = boid_locations[i][1];
	double z   = boid_locations[i][2];
	double pitch = boid_locations[i][3];
	double roll = boid_locations[i][4];
	double yaw = boid_locations[i][5];
	
	double fov = 125;

	double scale = 1;
	
	double Normal[3] = {0,1,0};
	rotate_vector(Normal,pitch,roll,yaw);
	
	if(informed) {
		informed_interact(Normal,x,y,z,i);
	} else {
		dumb_interact(Normal,x,y,z,i);
	}
	
	Accuracy_vec[0] += Normal[0];
	Accuracy_vec[1] += Normal[1];
	Accuracy_vec[2] += Normal[2];
	
	
// 	printf("%d: %.1f,%.1f,%.1f\n",i,Normal[0],Normal[1],Normal[2]);
	
	double cross[3] = {0,1,0};
	rotate_vector(cross,pitch,roll,yaw);
	
	// Calculate yaw angle
	double u[2] = {cross[0],  cross[1]};
	double v[2] = {Normal[0], Normal[1]};
	
	double angle = angle_from_vectors2d(u,v);
	
	if(v[0]-u[0] > 0 && angle == angle && angle < fov && angle > -fov)
	{
		yaw += -1*angle * rotate_speed;
	} else if(angle == angle){
		yaw += angle * rotate_speed;
	}
	
	u[0] = cross[0];
	u[1] = cross[2];
	v[0] = Normal[0];
	v[1] = Normal[2];
	
	angle = angle_from_vectors2d(u,v);
	
	if(v[1]-u[1] > 0 && angle == angle && angle < fov && angle > -fov)
	{
		roll += -1* angle * rotate_speed;
	} else if(angle == angle){
		roll += angle * rotate_speed;
	}
	
	u[0] = cross[1];
	u[1] = cross[2];
	v[0] = Normal[1];
	v[1] = Normal[2];
	
	angle = angle_from_vectors2d(u,v);
	
	if(v[0]-u[0] > 0 && angle == angle && angle < fov && angle > -fov)
	{
		pitch += -1*angle * rotate_speed;
	} else if(angle == angle){
		pitch += angle * rotate_speed;
	}
	
// 	printf("%d: %.1f,%.1f,%.1f\n",i,pitch,roll,yaw);
	
// 	Print("%.1f")
	
	glTranslated(x,y,z);
	glRotated(yaw,0,0,1);
	glRotated(roll,0,1,0);
	glRotated(pitch,1,0,0);
	glScaled(scale,scale,scale);
	

	
// 	rotate_vector(Normal,pitch,roll,yaw);
	boid_locations[i][0] = x + Normal[0] * speed;
	boid_locations[i][1] = y + Normal[1] * speed;
	boid_locations[i][2] = z + Normal[2] * speed;
	boid_locations[i][3] = pitch;
	boid_locations[i][4] = roll;
	boid_locations[i][5] = yaw;
	
	
	for(int j=0; j<3;j++)
	{
		if(boid_locations[i][j] > b_scaled) boid_locations[i][j] = -b_scaled;
		if(boid_locations[i][j] < -b_scaled) boid_locations[i][j] = b_scaled;
	}
	
	int d = 90;
	double r = 1;
	double g = 0.5;
	double b = 0;
	
	glBegin(GL_LINES);
	glVertex3d(Normal[0],Normal[1],Normal[2]);
	glVertex3d(Normal[0]*1.5,Normal[1]*1.5,Normal[2]*1.5);
	glEnd();
	

	for(int th=0;th<360;th+=d)
	{

		glRotated(th,0,1,0);
		
		glBegin(GL_POLYGON);
		glColor3f(0,0,1);
		if(informed) glColor3f(1,0,0);
		glVertex3d(-0.5,-0.5, 0.5);
		glVertex3d( 0.0, 1, 0.0);
		glVertex3d( 0.5,-0.5, 0.5);
		glEnd();

		if(th%180 == 0){
			r = 1-r;
		} else {
			b = 1-b;
		}
	}
	
	glBegin(GL_POLYGON);
	glColor3f(1,1,.5);
	glVertex3d(-0.5,-0.5, 0.5);
	glVertex3d(-0.5,-0.5,-0.5);
	glVertex3d( 0.5,-0.5,-0.5);
	glVertex3d( 0.5,-0.5, 0.5);
	glEnd();
   
   	glPopMatrix();
}

void draw_boids(double zh)
{
	int info=0;
	
	
	if(mode) {
	
		for(int i=0; i<2; i++) 
		{
		info=0;
		if(i<1) info =1;
		
		glPushMatrix();
		boid(i,info);
		glPopMatrix();
	
		}
	
	} else {
		
		for(int i=0; i<N; i++) {
			info=0;
			if(i<info_thresh) info =1;
		
			glPushMatrix();
			boid(i,info);
			glPopMatrix();
	
		}
	}
}


void display() {

	//  Erase the window and the depth buffer
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT|GL_STENCIL_BUFFER_BIT);
	//  Enable Z-buffering in OpenGL
	glEnable(GL_DEPTH_TEST);
	//  Undo previous transformations
	glLoadIdentity();

	glRotatef(ph,1,0,0);
	glRotatef(th,0,1,0);


	draw_boids(zh);
	draw_boarders();


	glColor3f(1,1,1);
   if (axes)
   {
      const double len=8.0;  //  Length of axes
      glBegin(GL_LINES);
      glVertex3d(0.0,0.0,0.0);
      glVertex3d(len,0.0,0.0);
      glVertex3d(0.0,0.0,0.0);
      glVertex3d(0.0,len,0.0);
      glVertex3d(0.0,0.0,0.0);
      glVertex3d(0.0,0.0,len);
      glEnd();
      //  Label axes
      glRasterPos3d(len,0.0,0.0);
      Print("X");
      glRasterPos3d(0.0,len,0.0);
      Print("Y");
      glRasterPos3d(0.0,0.0,len);
      Print("Z");
   }

	glColor3f(1,1,1);
	glWindowPos2i(5,5);
	
 
	double acclen = sqrt(pow(Accuracy_vec[0],2) + pow(Accuracy_vec[1],2) + pow(Accuracy_vec[2],2));
	Accuracy_vec[0] = Accuracy_vec[0]/acclen;
	Accuracy_vec[1] = Accuracy_vec[1]/acclen;
	Accuracy_vec[2] = Accuracy_vec[2]/acclen;
	

	
	double acc =( Accuracy_vec[0] +  Accuracy_vec[1] + Accuracy_vec[2])/3;
	
	Print("a: %.1f,p: %.1f, w: %.1f Informed: %.1f\%", r_inner, r_outer,w,(info_thresh/N)*100);
	Print(" Accuracy: %.2f",acc/.90);
	if(v)Print(" speed: %.1f,rotate_speed: %.1f", speed, rotate_speed);
	if(v)Print(" th: %d, ph: %d, zh %.1f",th,ph,zh);
	ErrCheck("display");
	glFlush();
	glutSwapBuffers();
}

/*
 *  The Special keys for the program. Currently the 1st person rotation does not work as intended
 */
void special(int key,int x,int y)
{
   //  Right arrow key - increase angle by 5 degrees
   if (key == GLUT_KEY_RIGHT) {
      	th += 5;
   }
   //  Left arrow key - decrease angle by 5 degrees
   else if (key == GLUT_KEY_LEFT) {
      	th -= 5;
   }
   //  Up arrow key - increase elevation by 5 degrees
   else if (key == GLUT_KEY_UP)
      ph += 5;
   //  Down arrow key - decrease elevation by 5 degrees
   else if (key == GLUT_KEY_DOWN)
      ph -= 5;
   //  Keep angles to +/-360 degrees
   th %= 360;
   ph %= 360;
//    //  Update projection
   Project(asp,dim);
   //  Tell GLUT it is necessary to redisplay the scene
   glutPostRedisplay();
}

/*
 *  Executes commands on key press. 
 */
void key(unsigned char ch,int x,int y)
{
// 	if(pitch > 360) pitch -=360;
// 	if(yaw > 360) 	yaw -=360;
// 	if(roll > 360)  roll -=360;
// 	if(pitch < -360) pitch +=360;
// 	if(yaw < -360) 	yaw +=360;
// 	if(roll < -360)  roll +=360;
   int shift = 1;
   double scale=1.5;
   //  Exit on ESC
   if (ch == 27)
      exit(0);
   //  Reset view angle
   else if (ch == '0') {
      th = ph = 5;
      speed = 0.05;
      rotate_speed=0.05;
      setup();
//       pitch=roll=yaw=0;


	}
   //  Switch display mode
   else if (ch == 'm' || ch == 'M') {
	  mode = 1-mode;
	  setup();

   } else if (ch=='C' || ch=='c') {
   		colors = 1-colors;
   } else if (ch=='k') {
   		stop = 1-stop;
   } else if(ch=='[') {
   		zh+=5;
   } else if(ch==']') {
   		zh-=5;
   } else if(ch=='a' || ch=='A') {
   		axes=1-axes;
   } else if(ch=='1') {
   		r_inner+=0.5;
   } else if(ch=='2') {
   		r_outer+=0.5;
   } else if(ch=='!') {
   		r_inner-=0.5;
   } else if(ch=='@') {
   		r_outer-=0.5;
   } else if(ch=='w') {
   		w+=0.1;
   } else if(ch=='W') {
   		w-=0.1;
   } else if(ch=='i') {
   		info_thresh+=1;
   } else if(ch=='I') {
   		info_thresh-=1;
   } else if(ch=='t') {
   		rotate_speed+=0.01;
   } else if(ch=='T') {
   		rotate_speed-=0.01;
   } else if(ch=='r') {
   		speed+=0.02;
   } else if(ch=='R') {
   		speed-=0.02;
   } else if(ch=='v') {
   		v = 1-v;
   }

   //  Reproject
   
   Project(asp,dim);
   //  Tell GLUT it is necessary to redisplay the scene
   glutPostRedisplay();
}

//Keep picture consistent when resizing
void reshape(int width,int height)
{
   //  Ratio of the width to the height of the window
   asp = (height>0) ? (double)width/height : 1;
   //  Set the viewport to the entire window
   glViewport(0,0, RES*width,RES*height);
   //  Set projection
   Project(asp,dim);
}

void idle()
{
   if(stop) {
	   double t = glutGet(GLUT_ELAPSED_TIME)/1000.0;
	   zh = fmod(90*t,360);
	   glutPostRedisplay();
   }
}

int main(int argc,char* argv[])
{
	setup();
    //Set up random function
   srand((unsigned int)time(NULL));
   //  Initialize GLUT and process user parameters
   glutInit(&argc,argv);
   //  Request double buffered, true color window with Z buffering at 600x600
   glutInitWindowSize(1200,1200);
   glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
   //  Create the window
   glutCreateWindow("Informed Leaders Simulation");
#ifdef USEGLEW
   //  Initialize GLEW
   if (glewInit()!=GLEW_OK) Fatal("Error initializing GLEW\n");
#endif
   //  Tell GLUT to call "idle" when there is nothing else to do
   glutIdleFunc(idle);
   //  Tell GLUT to call "display" when the scene should be drawn
   glutDisplayFunc(display);
   //  Tell GLUT to call "reshape" when the window is resized
   glutReshapeFunc(reshape);
   //  Tell GLUT to call "special" when an arrow key is pressed
   glutSpecialFunc(special);
   //  Tell GLUT to call "key" when a key is pressed
   glutKeyboardFunc(key);


   //  Pass control to GLUT so it can interact with the user
   glutMainLoop();
   return 0;
}



