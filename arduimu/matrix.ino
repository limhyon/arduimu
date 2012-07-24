// Quaternion integration: dq = (0.5*Qw*q)*dt;
//		double Qw[4][4] = {{   0,   -w[0],  -w[1], -w[2]},
//						   { w[0],     0,    w[2], -w[1]},
//						   { w[1],  -w[2],     0,   w[0]},
//						   { w[2],   w[1],  -w[0],    0 }};
void Make_SkewM_4by4_from_Vec(float vec[3],float mat[4][4])
{
  mat[0][0] = 0;
  mat[1][1] = 0;
  mat[2][2] = 0;
  mat[3][3] = 0;
  
  mat[0][1] = -vec[0];
  mat[0][2] = -vec[1];
  mat[0][3] = -vec[2];
  
  mat[1][0] = vec[0];
  mat[1][2] = vec[2];
  mat[1][3] = -vec[1];
  
  mat[2][0] = vec[1];
  mat[2][1] = -vec[2];
  mat[2][3] = vec[0];
  
  mat[3][0] = vec[2];
  mat[3][1] = vec[1];
  mat[3][2] = -vec[0];
}

void Quaternion_Multiply(float q[4],float w[3],float deltaT)
{
  float dq[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  float Qw[4][4];

    Make_SkewM_4by4_from_Vec(w,Qw);
    
    for (int i=0; i < 4; i++)
    {
  	dq[i] = 0.5f*(Qw[i][0]*q[0] + Qw[i][1]*q[1] + Qw[i][2]*q[2] + Qw[i][3]*q[3])*(deltaT);
  	q[i] += dq[i];
    }
    
    float norm = 1.0 / sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    
    for (int i=0; i < 4; i++) q[i] = q[i]*norm;
}

void Quat2DCM(float q[4],float R[3][3])
{
        float w = q[0];
	float x = q[1];
	float y = q[2];
	float z = q[3];

	float n = sqrt(w*w + x*x + y*y + z*z);
	
	w = w/n;
	x = x/n;
	y = y/n;
	z = z/n;

	float xx = x*x, xy = x*y, xz = x*z, xw = x*w;
	float yy = y*y, yz = y*z, yw = y*w;
	float zz = z*z, zw = z*w;
	float ww = w*w;

	R[0][0] = 1 - 2*(yy + zz);      R[0][1] =     2*(xy - zw);      R[0][2] =     2*(xz + yw);
	R[1][0] =     2*(xy + zw);	R[1][1] = 1 - 2*(xx + zz);	R[1][2] =     2*(yz - xw);
	R[2][0] =     2*(xz - yw);	R[2][1] =     2*(yz + xw);	R[2][2] = 1 - 2*(xx + yy);
}

void DCM2Quat(float R[3][3], float q[4])
{
  q[3] = sqrt( R[0][0] + R[1][1] + R[2][2] + 1 );
q[0] = (R[1][2] - R[2][1]) / 4*q[3];
q[1] = (R[2][0] - R[0][2]) / 4*q[3];
q[2] = (R[0][1] - R[1][0]) / 4*q[3];
}

/**************************************************/
//Multiply two 3x3 matrixs. This function developed by Jordi can be easily adapted to multiple n*n matrix's. (Pero me da flojera!). 
void Matrix_Multiply(float a[3][3], float b[3][3],float mat[3][3])
{
  float op[3]; 
  for(int x=0; x<3; x++)
  {
    for(int y=0; y<3; y++)
    {
      for(int w=0; w<3; w++)
      {
       op[w]=a[x][w]*b[w][y];
      } 
      mat[x][y]=0;
      mat[x][y]=op[0]+op[1]+op[2];
      
      float test=mat[x][y];
    }
  }
}


