float *q_m(float q_sol[], float q_l[], float q_r[]){
    q_sol[0] = q_l[0]*q_r[0]-q_l[1]*q_r[1]-q_l[2]*q_r[2]-q_l[3]*q_r[3];
    q_sol[1] = q_l[0]*q_r[1]+q_l[1]*q_r[0]+q_l[2]*q_r[3]-q_l[3]*q_r[2];
    q_sol[2] = q_l[0]*q_r[2]+q_l[2]*q_r[0]+q_l[3]*q_r[1]-q_l[1]*q_r[3];
    q_sol[3] = q_l[0]*q_r[3]+q_l[3]*q_r[0]+q_l[1]*q_r[2]-q_l[2]*q_r[1];

    return q_sol;
}