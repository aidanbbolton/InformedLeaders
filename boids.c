#include "WGP.h"

#define N 5

void boid(float x, float y, float z, float rotation, float scale)
{

	glPushMatrix();

	glTranslated(x,y,z);
	glRotated(rotation,0,0,1);
	glScaled(scale,scale,scale);
	
	int d = 90;
	float r = 1;
	float g = 0.5;
	float b = 0;

	for(int th=0;th<360;th+=d)
	{

		glRotated(th,0,1,0);
		
		glBegin(GL_POLYGON);
		glColor3f(r,g,b);
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

void draw_boids(int n, double zh)
{

	float boid_locations[N][3];
	
	for(int i=0; i<N; i++)
	{
		for(int j=0; j<3; j++)
		{
			
			if(j==1)
			{
				boid_locations[i][j] = i;
			} else {
				boid_locations[i][j] = 0;
			}
		}
	}
	
	for(int i=0; i<N; i++) {
	
		float x = boid_locations[i][0];
		float y = boid_locations[i][1];
		float z = boid_locations[i][2];
	
		glPushMatrix();

		boid(x,y,z,270,1);

		glPopMatrix();

	}
	
	for(int i=0; i<N; i++)
	{
		for(int j=0; j<3; j++)
		{
			if(j==0)
			{
				boid_locations[i][j] = boid_locations[i][j] + 1;
			}
		}
	}
}