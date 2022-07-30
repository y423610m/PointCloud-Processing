#include <string>
#include <vector>
#include <iostream>
#include <cmath>

#include <stdio.h>
#include <winsock2.h>
#include <ws2tcpip.h>

#define MAX_BUF_SIZE    256      //受信するメッセージの最大文字数

class LabViewInterface {
private:
	WSADATA wsaData;                //winsock情報を保管
	struct sockaddr_in server_addr; //サーバーのipアドレスとポート番号情報格納
	SOCKET sock;                    //クライアントのソケット
	char buf[MAX_BUF_SIZE];         //受信したメッセージを格納

	bool constructed = false;

	std::vector<float> vf;
	int cnt_ = 0;

public:
	LabViewInterface(PCSTR IP, int PORT);
	~LabViewInterface();
	void update();
	std::vector<float> get_tip_pose(){ return vf; };

};