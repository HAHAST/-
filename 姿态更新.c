



void POS_UPDATE(float w[3])  //姿态更新
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
