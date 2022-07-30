#include <string>
#include <vector>
#include <iostream>
#include <cmath>

#include <stdio.h>
#include <winsock2.h>
#include <ws2tcpip.h>

#define MAX_BUF_SIZE    256      //��M���郁�b�Z�[�W�̍ő啶����

class LabViewInterface {
private:
	WSADATA wsaData;                //winsock����ۊ�
	struct sockaddr_in server_addr; //�T�[�o�[��ip�A�h���X�ƃ|�[�g�ԍ����i�[
	SOCKET sock;                    //�N���C�A���g�̃\�P�b�g
	char buf[MAX_BUF_SIZE];         //��M�������b�Z�[�W���i�[

	bool constructed = false;

	std::vector<float> vf;
	int cnt_ = 0;

public:
	LabViewInterface(PCSTR IP, int PORT);
	~LabViewInterface();
	void update();
	std::vector<float> get_tip_pose(){ return vf; };

};