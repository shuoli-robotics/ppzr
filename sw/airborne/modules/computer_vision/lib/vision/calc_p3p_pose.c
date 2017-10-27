


void compute_p3p_pose(p3p_phi,p3p_theta,p3p_psi){
        //draw_gate_color(img, best_gate, blue_color);
	//draw_gate_polygon(img,points_x,points_y,blue_color);
	//draw_gate_polygon(img,best_gate.x_corners,best_gate.y_corners,blue_color);
	
	//vector and matrix declarations
	  struct FloatVect3 gate_point_0,gate_point_1,gate_point_2,
	  p3p_pos_sol_0,p3p_pos_sol_1,p3p_pos_sol_2,p3p_pos_sol_3,p3p_pos_test;
	  struct FloatMat33 R_mat_0,R_mat_1,R_mat_2,R_mat_3, R_test;
	  struct FloatEulers R_eulers;
	  
	  struct FloatVect3 p3p_pos_sol[4];
	  struct FloatVect3 gate_shift_points[4];
	  struct FloatVect3 gate_shift_vec[4];
	  struct FloatMat33 R_mat[4];
	  //ransac 
	  int ransac_error[4];
	  float ransac_rep_error[4];
	  struct FloatVect3 ransac_pos[4];
	  struct FloatMat33 ransac_R_mat[4];
	  
	  VECT3_ASSIGN(gate_points[0], 4.2000,0.3000, -1.9000);
	  VECT3_ASSIGN(gate_points[1], 4.2000,1.3000, -1.9000);
	  VECT3_ASSIGN(gate_points[2], 4.2000,1.3000, -0.9000);
	  VECT3_ASSIGN(gate_points[3], 4.2000,0.3000, -0.9000);
	
	//DEBUG----------------------------------------------------------------
	
	  VECT3_ASSIGN(p3p_pos_test, 2.295645,0.877159, -1.031926);
	  
	  MAT33_ELMT(R_test, 0, 0) = 0.999867;//(row,column)
	  MAT33_ELMT(R_test, 0, 1) =  -0.015745;
	  MAT33_ELMT(R_test, 0, 2) = -0.004201;

	  MAT33_ELMT(R_test, 1, 0) = 0.015464;//(row,column)
	  MAT33_ELMT(R_test, 1, 1) = 0.998070;
	  MAT33_ELMT(R_test, 1, 2) = -0.060151;

	  MAT33_ELMT(R_test, 2, 0) = 0.005140;//(row,column)
	  MAT33_ELMT(R_test, 2, 1) = 0.060078;
	  MAT33_ELMT(R_test, 2, 2) = 0.998180;
// 	
// 	  MAT33_ELMT(R_test, 0, 0) = 1;//(row,column)
// 	  MAT33_ELMT(R_test, 0, 1) = 0;
// 	  MAT33_ELMT(R_test, 0, 2) = 0;
// 
// 	  MAT33_ELMT(R_test, 1, 0) = 0;//(row,column)
// 	  MAT33_ELMT(R_test, 1, 1) = 1;
// 	  MAT33_ELMT(R_test, 1, 2) = 0;
// 
// 	  MAT33_ELMT(R_test, 2, 0) = 0;//(row,column)
// 	  MAT33_ELMT(R_test, 2, 1) = 0;
// 	  MAT33_ELMT(R_test, 2, 2) = 1;
	  
	int x_bp_corners[4];
	int y_bp_corners[4];
	float x_f_corners[4];
	float y_f_corners[4];
	float x_gate_corners[4];
	float y_gate_corners[4];
	
	for(int i = 0;i<4;i++)
	  {
	    float x_bp;
	    float y_bp;
	    back_proj_points(&gate_points[i],&p3p_pos_test,&R_test,&x_bp,&y_bp);
	    x_f_corners[i] = x_bp;
	    y_f_corners[i] = y_bp;
// 	    debug_1 = x_bp;
// 	    debug_2 = y_bp;
	  }
	
	//DEBUG---------------------------------------------------------------
	
	//Undistort fisheye points
	int f_fisheye = 168;
	float k_fisheye = 1.085;
	float reprojection_error[4];
	for(int i = 0;i<4;i++)
	{
	  float undist_x, undist_y;
	  //debug_1 = (float)best_gate.x_corners[i];// best_gate.y_corners[i]
	  //debug_2 = (float)best_gate.y_corners[i];//
	  //undist_y = 5;//(float)best_gate.y_corners[i];
	  float princ_x = 157.0;
	  float princ_y = 32.0;
	  undistort_fisheye_point(best_gate.x_corners[i] ,best_gate.y_corners[i],&undist_x,&undist_y,f_fisheye,k_fisheye,princ_x,princ_y);
	  
	  x_gate_corners[i] = undist_x+157.0;
	  y_gate_corners[i] = undist_y+32.0;
	  
	  draw_cross(img,((int)x_gate_corners[i]),((int)y_gate_corners[i]),green_color);
	  vec_from_point_ned(undist_x, undist_y, f_fisheye,&gate_vectors[i]);
	  
	   //debug_1 = undist_x;
 	  //debug_2 = undist_y;
	  
// 	  draw_cross(img,(int)x_f_corners[i],(int)y_f_corners[i],green_color);
// 	  vec_from_point_ned(x_f_corners[i]-157,y_f_corners[i]-32, f_fisheye,&gate_vectors[i]);
	  
	 // //vec_from_point_ned(x_f_corners[i]-157,y_f_corners[i]-32, f_fisheye,&vec_temp1);
// 	  
// 	  printf("vec_from_point:\n");
// 	  print_vec(vec_temp1);
// 	  
// 	  VECT3_DIFF(vec_temp1,gate_points[i],p3p_pos_sol_0);
// 	  double norm = sqrt(VECT3_NORM2(vec_temp1));
// 	  VECT3_SDIV(gate_vectors[i], vec_temp1, norm);
// 	  
// 	  printf("gate_vectors[%d]:\n",i);
// 	  print_vec(gate_vectors[i]);
	  
	  
// 	  debug_1 = gate_vectors[i].x;
// 	  debug_2 = gate_vectors[i].y;
// 	  debug_3 = gate_vectors[i].z;
	  
	}
	
	int in_array[4] = {1, 2, 3, 4};
	int shift_array[4];
	int index = 0;
	//circ shift for checking which triplet of points gives best solution after back projection
	for(int s = 0;s<4;s++)
	{
	  for(int i = 0; i < 4;i++)
	  {
	    index = (i+s) % 4;
	    shift_array[index] = in_array[i];
	    gate_shift_points[index] = gate_points[i];
	    gate_shift_vec[index] = gate_vectors[i];
	  }
	  printf("shift_n:%d\n",s);
	  printf("shift_array{%d, %d, %d, %d} \n",shift_array[0],shift_array[1],shift_array[2],shift_array[3]);
	  printf("gate_shift_points[%d] x:%f y:%f z%f\n",0,gate_shift_points[0].x,gate_shift_points[0].y,gate_shift_points[0].z);
	  printf("gate_shift_points[%d] x:%f y:%f z%f\n",1,gate_shift_points[1].x,gate_shift_points[1].y,gate_shift_points[1].z);
	  printf("gate_shift_points[%d] x:%f y:%f z%f\n",2,gate_shift_points[2].x,gate_shift_points[2].y,gate_shift_points[2].z);
	  printf("gate_shift_points[%d] x:%f y:%f z%f\n",3,gate_shift_points[3].x,gate_shift_points[3].y,gate_shift_points[3].z);
	
	//}
	
	
	 //p3p algorithm
	
	  
// 	  P3p_computePoses(&gate_points[0],&gate_points[1],&gate_points[2],
// 			   &gate_vectors[0],&gate_vectors[1],&gate_vectors[2],
// 			   &p3p_pos_sol_0,&p3p_pos_sol_1,&p3p_pos_sol_2,&p3p_pos_sol_3,
// 			   &R_mat_0,&R_mat_1,&R_mat_2,&R_mat_3);
	  
// 	  P3p_computePoses(&gate_points[0],&gate_points[1],&gate_points[2],
// 			   &gate_vectors[0],&gate_vectors[1],&gate_vectors[2],
// 			   &p3p_pos_sol[0],&p3p_pos_sol[1],&p3p_pos_sol[2],&p3p_pos_sol[3],
// 			   &R_mat[0],&R_mat[1],&R_mat[2],&R_mat[3]);

	  P3p_computePoses(&gate_shift_points[0],&gate_shift_points[1],&gate_shift_points[2],
			   &gate_shift_vec[0],&gate_shift_vec[1],&gate_shift_vec[2],
			   &p3p_pos_sol[0],&p3p_pos_sol[1],&p3p_pos_sol[2],&p3p_pos_sol[3],
			   &R_mat[0],&R_mat[1],&R_mat[2],&R_mat[3]);
	
	  float_eulers_of_rmat(&R_eulers,&R_mat_0);
	  
	  //for all 4 solutions
	  //{
	  //proj_point(p3p_pos_sol_0,p3p_pos_sol_1,p3p_pos_sol_2,R_mat) //project 4 points based on position and rotation
	  //}
	  //best_solution_p3p()//sort error list and chose respective solution pos and R_mat
	  //R_mat_to_angle()//from paparazzi
	  
// 	  VECT3_ASSIGN(p3p_pos_sol_0, 1.5000,0.8000, -1.4000);
//	  VECT3_ASSIGN(p3p_pos_sol_0, 2.295645,0.877159, -1.031926);
	  
// 	  MAT33_ELMT(R_mat_0, 0, 0) = 1;//(row,column)
// 	  MAT33_ELMT(R_mat_0, 0, 1) = 0;
// 	  MAT33_ELMT(R_mat_0, 0, 2) = 0;
// 
// 	  MAT33_ELMT(R_mat_0, 1, 0) = 0;//(row,column)
// 	  MAT33_ELMT(R_mat_0, 1, 1) = 1;
// 	  MAT33_ELMT(R_mat_0, 1, 2) = 0;
// 
// 	  MAT33_ELMT(R_mat_0, 2, 0) = 0;//(row,column)
// 	  MAT33_ELMT(R_mat_0, 2, 1) = 0;
// 	  MAT33_ELMT(R_mat_0, 2, 2) = 1;
	  
// 	  MAT33_ELMT(R_mat_0, 0, 0) = 0.999867;//(row,column)
// 	  MAT33_ELMT(R_mat_0, 0, 1) =  -0.015745;
// 	  MAT33_ELMT(R_mat_0, 0, 2) = -0.004201;
// 
// 	  MAT33_ELMT(R_mat_0, 1, 0) = 0.015464;//(row,column)
// 	  MAT33_ELMT(R_mat_0, 1, 1) = 0.998070;
// 	  MAT33_ELMT(R_mat_0, 1, 2) = -0.060151;
// 
// 	  MAT33_ELMT(R_mat_0, 2, 0) = 0.005140;//(row,column)
// 	  MAT33_ELMT(R_mat_0, 2, 1) = 0.060078;
// 	  MAT33_ELMT(R_mat_0, 2, 2) = 0.998180;
	  
	  
	  /*0.999867, -0.015745, -0.004201
0.015464, 0.998070, -0.060151
0.005140, 0.060078, 0.998180
Position solution nr:1 /// x:2.295645 y:0.877159 z-1.031926*/
	  
// 	  	printf("R_mat_0:\n");
//  		print_mat(R_mat_0);
// 		printf("Position solution nr:%d /// x:%f y:%f z%f\n",0,p3p_pos_sol_0.x,p3p_pos_sol_0.y,p3p_pos_sol_0.z);
// 		float_eulers_of_rmat(&R_eulers,&R_mat_0);
// 		printf("Eulers_0 Phi:%f \n Theta: %f \n Psi: %f \n",(R_eulers.phi*57),(R_eulers.theta*57),(R_eulers.psi*57));
// 		
// 		printf("R_mat_1:\n");
//  		print_mat(R_mat_1);
// 		printf("Position solution nr:%d /// x:%f y:%f z%f\n",1,p3p_pos_sol_1.x,p3p_pos_sol_1.y,p3p_pos_sol_1.z);
// 		float_eulers_of_rmat(&R_eulers,&R_mat_1);
// 		printf("Eulers_1 Phi:%f \n Theta: %f \n Psi: %f \n",(R_eulers.phi*57),(R_eulers.theta*57),(R_eulers.psi*57));
// 		
// 		printf("R_mat_2:\n");
//  		print_mat(R_mat_2);
// 		printf("Position solution nr:%d /// x:%f y:%f z%f\n",2,p3p_pos_sol_2.x,p3p_pos_sol_2.y,p3p_pos_sol_2.z);
// 		float_eulers_of_rmat(&R_eulers,&R_mat_2);
// 		printf("Eulers_2 Phi:%f \n Theta: %f \n Psi: %f \n",(R_eulers.phi*57),(R_eulers.theta*57),(R_eulers.psi*57));
// 		
// 		
// 		printf("R_mat_3:\n");
//  		print_mat(R_mat_3);
// 		printf("Position solution nr:%d /// x:%f y:%f z%f\n",3,p3p_pos_sol_3.x,p3p_pos_sol_3.y,p3p_pos_sol_3.z);
// 		float_eulers_of_rmat(&R_eulers,&R_mat_3);
// 		printf("Eulers_3 Phi:%f \n Theta: %f \n Psi: %f \n",(R_eulers.phi*57),(R_eulers.theta*57),(R_eulers.psi*57));
// 		

	  	
// 		printf("Position solution nr:%d +++ x:%f y:%f z%f\n",0,p3p_pos_sol[0].x,p3p_pos_sol[0].y,p3p_pos_sol[0].z);
// 		print_mat(R_mat[0]);
// 		printf("Position solution nr:%d +++ x:%f y:%f z%f\n",1,p3p_pos_sol[1].x,p3p_pos_sol[1].y,p3p_pos_sol[1].z);
// 		print_mat(R_mat[1]);
// 		printf("Position solution nr:%d +++ x:%f y:%f z%f\n",2,p3p_pos_sol[2].x,p3p_pos_sol[2].y,p3p_pos_sol[2].z);
// 		print_mat(R_mat[2]);
// 		printf("Position solution nr:%d +++ x:%f y:%f z%f\n",3,p3p_pos_sol[3].x,p3p_pos_sol[3].y,p3p_pos_sol[3].z);
// 		print_mat(R_mat[3]);
		
	
	  reprojection_error[0] = 0;
	  for(int i = 0;i<4;i++)
	  {
	    float x_bp;
	    float y_bp;
	    back_proj_points(&gate_points[i],&p3p_pos_sol[0],&R_mat[0],&x_bp,&y_bp);
	    x_bp_corners[i] = (int)x_bp;
	    y_bp_corners[i] = (int)y_bp;
	    reprojection_error[0] += euclidean_distance(x_gate_corners[i], x_bp_corners[i],y_gate_corners[i],y_bp_corners[i]);
// 	    debug_1 = x_bp;
// 	    debug_2 = y_bp;
	  }
	  printf("reprojection_error[0]:%f\n",reprojection_error[0]);
	  //draw_gate_polygon(img,x_bp_corners,y_bp_corners,blue_color);
	  
	  reprojection_error[1] = 0;
	  for(int i = 0;i<4;i++)
	  {
	    float x_bp;
	    float y_bp;
	    back_proj_points(&gate_points[i],&p3p_pos_sol[1],&R_mat[1],&x_bp,&y_bp);
	    x_bp_corners[i] = (int)x_bp;
	    y_bp_corners[i] = (int)y_bp;
	    reprojection_error[1] += euclidean_distance(x_gate_corners[i], x_bp_corners[i],y_gate_corners[i],y_bp_corners[i]);
	  }
	  printf("reprojection_error[1]:%f\n",reprojection_error[1]);
	  //raw_gate_polygon(img,x_bp_corners,y_bp_corners,blue_color);
	  
	  reprojection_error[2] = 0;
	  for(int i = 0;i<4;i++)
	  {
	    float x_bp;
	    float y_bp;
	    back_proj_points(&gate_points[i],&p3p_pos_sol[2],&R_mat[2],&x_bp,&y_bp);
	    x_bp_corners[i] = (int)x_bp;
	    y_bp_corners[i] = (int)y_bp;
	    reprojection_error[2] += euclidean_distance(x_gate_corners[i], x_bp_corners[i],y_gate_corners[i],y_bp_corners[i]);
	  }
	  printf("reprojection_error[2]:%f\n",reprojection_error[2]);
	  //draw_gate_polygon(img,x_bp_corners,y_bp_corners,blue_color);
	  
	  reprojection_error[3] = 0;
	  for(int i = 0;i<4;i++)
	  {
	    float x_bp;
	    float y_bp;
	    back_proj_points(&gate_points[i],&p3p_pos_sol[3],&R_mat[3],&x_bp,&y_bp);
	    x_bp_corners[i] = (int)x_bp;
	    y_bp_corners[i] = (int)y_bp;
	    reprojection_error[3] += euclidean_distance(x_gate_corners[i], x_bp_corners[i],y_gate_corners[i],y_bp_corners[i]);
	  }
	  printf("reprojection_error[3]:%f\n",reprojection_error[3]);
	  //draw_gate_polygon(img,x_bp_corners,y_bp_corners,blue_color);
	  
	 
	  int min_loc = find_minimum(reprojection_error);
	  ransac_rep_error[s] = reprojection_error[min_loc];
	  ransac_error[s] = min_loc;
	  ransac_pos[s] = p3p_pos_sol[min_loc];
	  ransac_R_mat[s] = R_mat[min_loc];
	  
	}
	  
	  int best_loc = find_minimum(ransac_error);
	  //best_loc = 3;
	  printf("Minimum error at solution:%d\n",best_loc);
	  printf("Minimum error in pixels:%f\n",ransac_rep_error[best_loc]);
	  printf("POSITION:\n X:%f\n Y:%f\n Z:%f\n",ransac_pos[best_loc].x,ransac_pos[best_loc].y,ransac_pos[best_loc].z);
	  	 
	  
	  for(int i = 0;i<4;i++)
	  {
	    float x_bp;
	    float y_bp;
	    back_proj_points(&gate_points[i],&ransac_pos[best_loc],&ransac_R_mat[best_loc],&x_bp,&y_bp);
	    x_bp_corners[i] = (int)x_bp;
	    y_bp_corners[i] = (int)y_bp;
	  }
	  
	  
	  //gate size check by calculating total length of elements between polygon gate
	  //then determining error bound as function of gate size: larger gate -> higher error bound
	  float gate_size_polygon =  euclidean_distance(x_gate_corners[0], x_gate_corners[1],y_gate_corners[0],y_gate_corners[1]);
	  gate_size_polygon +=  euclidean_distance(x_gate_corners[1], x_gate_corners[2],y_gate_corners[1],y_gate_corners[2]);
	  gate_size_polygon +=  euclidean_distance(x_gate_corners[2], x_gate_corners[3],y_gate_corners[2],y_gate_corners[3]);
	  gate_size_polygon +=  euclidean_distance(x_gate_corners[3], x_gate_corners[0],y_gate_corners[3],y_gate_corners[0]);
	  
	  float error_factor =  ransac_rep_error[best_loc]/gate_size_polygon;//ration between gate size and reprojection error
	  printf("error_factor:%f\n",error_factor);
	  if(error_factor < 0.07)
	  {
	    draw_gate_polygon(img,x_bp_corners,y_bp_corners,blue_color);//if low enough plot green gate
	  
	    float_eulers_of_rmat(&R_eulers,&ransac_R_mat[best_loc]);
	    printf("Eulers Phi:%f \n Theta: %f \n Psi: %f \n",(R_eulers.phi*57),(R_eulers.theta*57),(R_eulers.psi*57));
	  }
	  
	  //if(ransac_rep_error[best_loc] < 25) draw_gate_polygon(img,x_bp_corners,y_bp_corners,blue_color);
// 	  printf("POSITION:\n X:%f\n Y:%f\n Z:%f\n",p3p_pos_sol[min_loc].x,p3p_pos_sol[min_loc].y,p3p_pos_sol[min_loc].z);
	  //print best gate
// 	  for(int i = 0;i<4;i++)
// 	  {
// 	    float x_bp;
// 	    float y_bp;
// 	    back_proj_points(&gate_points[i],&p3p_pos_sol[min_loc],&R_mat[min_loc],&x_bp,&y_bp);
// 	    x_bp_corners[i] = (int)x_bp;
// 	    y_bp_corners[i] = (int)y_bp;
// 	  }
// 	  draw_gate_polygon(img,x_bp_corners,y_bp_corners,blue_color);
	 
	
	    //error tests
	// reference vectors
	
// 	vec_ver_1.x = 200;
// 	vec_ver_1.y = -100;
// 	vec_ver_1.z = 72;
// 	
// 	vec_ver_2.x = 200;
// 	vec_ver_2.y = 0;
// 	vec_ver_2.z = 72;
// 	
// 	vec_ver_3.x = 200;
// 	vec_ver_3.y = 0;
// 	vec_ver_3.z = -28;
// 	
// 	vec_ver_4.x = 200;
// 	vec_ver_4.y = -100;
// 	vec_ver_4.z = -28;
// 	
// 	double dot = VECT3_DOT_PRODUCT(vec_ver_1, gate_vectors[0]);
// 	float norm_a = sqrtf(VECT3_NORM2(vec_ver_1));
// 	float norm_b = sqrtf(VECT3_NORM2(gate_vectors[0]));
// 	
// 	float angle_error_dot = acosf((float)dot/(norm_a*norm_b));
// 	debug_1 = angle_error_dot*57;
// 	
// 	 dot = VECT3_DOT_PRODUCT(vec_ver_2, gate_vectors[1]);
// 	 norm_a = sqrtf(VECT3_NORM2(vec_ver_2));
// 	 norm_b = sqrtf(VECT3_NORM2(gate_vectors[1]));
// 	
// 	 angle_error_dot = acosf((float)dot/(norm_a*norm_b));
// 	 debug_2 = angle_error_dot*57;
// 	 
// 	 dot = VECT3_DOT_PRODUCT(vec_ver_3, gate_vectors[2]);
// 	 norm_a = sqrtf(VECT3_NORM2(vec_ver_3));
// 	 norm_b = sqrtf(VECT3_NORM2(gate_vectors[2]));
// 	
// 	 angle_error_dot = acosf((float)dot/(norm_a*norm_b));
// 	 debug_3 = angle_error_dot*57;
	
}
	
    
