

void EtoQ(float *s, float *x, float *y, float *z, float pitch, float roll, float yaw)  //欧拉角转四元数
{
	float norm;

	s = cos(roll / 2)*cos(pitch / 2)*cos(yaw / 2) - sin(yaw / 2)*sin(roll / 2)*sin(pitch / 2);
	x = cos(roll / 2)*sin(pitch / 2)*cos(yaw / 2) - sin(roll / 2)*cos(pitch / 2)*sin(yaw / 2);
	y = cos(roll / 2)*sin(pitch / 2)*sin(yaw / 2) + sin(roll / 2)*cos(pitch / 2)*cos(yaw / 2);
	z = cos(roll / 2)*cos(pitch / 2)*sin(yaw / 2) + sin(roll / 2)*sin(pitch / 2)*cos(yaw / 2);

	norm = sqrt(s*s + x * x + y * y + z * z);
	s = s / norm;
	x = x / norm;
	y = y / norm;
	z = z / norm;
}

void QtoE(float *pitch, float *roll, float *yaw, float s, float x, float y, float z) //四元数转欧拉角
{
	pitch = asin(2 * (z*y + s * x));
	yaw = atan2(2 * (z*s - y * x), (1 - 2 * x*x - 2 * z*z));
	roll = atan2(2 * (s*y - x * z), (1 - 2 * y*y - 2 * x*x));
}

void updata(float w[3])  //姿态更新
{
	float cup0, cup1, cup2, cup3, norm;
	cup0 = s - 0.5*(w[0]*x + w[1]*y + w[2]*z)*T;
	cup1 = x + 0.5*(w[0]*s + w[2]*y - w[1]*z)*T;
	cup2 = y + 0.5*(w[1]*s - w[2]*x + w[0]*z)*T;
	cup3 = z + 0.5*(w[2]*s + w[1]*x - w[0]*y)*T;
	norm = sqrt(cup0*cup0 + cup1*cup1 + cup2*cup2 + cup3*cup3);
	s = cup0 / norm;
	x = cup1 / norm;
	y = cup2 / norm;
	z = cup3 / norm;
}

//卡尔曼滤波 _(:з」∠)_
void KF(){
	
}
