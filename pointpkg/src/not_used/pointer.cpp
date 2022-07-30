#include <pointer.h>
#include <iostream>
#include <time.h>
#include <stdlib.h>


void Pointer::initialize(int ClientID) {
	simxGetObjectHandle(ClientID, "R_tip", &Rtt, simx_opmode_blocking);
	simxGetObjectHandle(ClientID, "L_tip", &Ltt, simx_opmode_blocking);
	simxGetObjectHandle(ClientID, "R_tip_help", &Rtthelp, simx_opmode_blocking);
	simxGetObjectHandle(ClientID, "L_tip_help", &Ltthelp, simx_opmode_blocking);
	//std::cout << Rtt << Ltt << Rtthelp << Ltthelp << std::endl;
}

void Pointer::loop(int ClientID) {
	srand(clock());

	const int size = 128;
	float output[3 * size*size];
	int color[3 * size*size];


	//if (count % 4 == 0) {
		double start = clock();
		int result_input = simxCallScriptFunction(ClientID, "RealSense_Depth", sim_scripttype_childscript, "MyInputData", 0, NULL, 0, NULL, 0, NULL, 0, NULL, 0, NULL, &number, &input, NULL, NULL, NULL, NULL, simx_opmode_blocking);
		double first = clock();

		count_pc = 0;
		
		simxGetObjectPosition(ClientID, Rtt, -1, position_Rtt, simx_opmode_oneshot);
		simxGetObjectPosition(ClientID, Ltt, -1, position_Ltt, simx_opmode_oneshot);
		simxGetObjectPosition(ClientID, Rtthelp, -1, position_Rtthelp, simx_opmode_oneshot);
		simxGetObjectPosition(ClientID, Ltthelp, -1, position_Ltthelp, simx_opmode_oneshot);

		zR = 0.5 * (position_Rtt[2] + position_Rtthelp[2]);
		zL = 0.5 * (position_Ltt[2] + position_Ltthelp[2]);
		//std::cout << position_Rtt[0] << "     " << position_Rtt[1]<<""<<position_Rtt[2]<<std::endl;
		//std::cout << zR << "     " << zL << std::endl;
		

		for (int i = 1; i <= size; i++) {
			for (int j = 1; j <= size; j++) {
				position[2] = dist_min + (dist_max - dist_min) * input[(i - 1) * size + (j - 1)] + 0.004 * (rand() % 100 - 50) / 100.0;//z
				position[0] = 1.03 * position[2] * tan(-angle / 2 + angle * (j - 1) / (size - 1));//x
				position[1] = 1.0 * position[2] * tan(-angle / 2 + angle * (i - 1) / (size - 1));//y
				//position[2] += 0.01 * (rand() % 100 - 50) / 100;
								
				output[3 * count_pc] = position[0] + dx;
				output[3 * count_pc + 1] = 1.04 * (position[1] * cos(angle_camera) + position[2] * sin(angle_camera)) + dy;
				output[3 * count_pc + 2] = position[1] * sin(angle_camera) - position[2] * cos(angle_camera) + dz;



				
				//restrict volume
				if (abs(output[3 * count_pc + 1]) < 0.15) {//y
					if (abs(output[3 * count_pc] - dx) < 0.15) {//x
						if (output[3 * count_pc + 2] > 0.096) {//z
							//RGB
							//color[3 * count_pc ] = (int)((output[3 * count_pc + 2]-0.096 )* 1200) % 255;
							color[3 * count_pc] = (int)((output[3 * count_pc + 1] + 0.15) * 800) % 255;

							color[3 * count_pc + 1] = 255-(int)((output[3 * count_pc + 1] + 0.15) * 800) % 255;
							color[3 * count_pc + 1] = 160;

							color[3 * count_pc + 2] = 255-(int)((output[3 * count_pc + 2] - 0.096) * 1500) % 255;
							//color[3 * count_pc + 2] = (int)((output[3 * count_pc + 1] + 0.15) * 800) % 255;
							/*
							//remove arm point
							if (output[3 * count_pc] - dx > 0) {//right
								if (output[3 * count_pc + 2]<zR) {
									count_pc++;
								}
							}
							else {
								if (output[3 * count_pc + 2] < zL) {
									count_pc++;
								}
							}
							*/
					//	output[3 * count_pc + 2] += 0.008 * (rand() % 100 - 50) / 100;
							count_pc++;
							
						}
					}
				}	
				
				

				//count_pc++;

			}
		}
		std::cout << count_pc << std::endl;
		//double second = clock();
		//std::cout << second << std::endl;
		//int result_output = simxCallScriptFunction(ClientID, "RealSense_Depth", sim_scripttype_childscript, "MyOutputData", 0,NULL, 3 * count_pc, output, 0, NULL, 0, NULL, 0, NULL, 0, NULL, NULL, NULL, NULL, NULL, simx_opmode_blocking);
		int result_output = simxCallScriptFunction(ClientID, "RealSense_Depth", sim_scripttype_childscript, "MyOutputData", 3 * count_pc, color, 3 * count_pc, output, 0, NULL, 0, NULL, 0, NULL, 0, NULL, NULL, NULL, NULL, NULL, simx_opmode_blocking);
		//double end = clock();
		//std::cout <<"data       "<< (double)(first - start) <<"  "<< (double)(second-first) <<"  "<< (double)(end-second) << std::endl;

	//}
	count++;
}



