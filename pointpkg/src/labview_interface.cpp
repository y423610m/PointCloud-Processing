#include "labview_interface.h"
using namespace std;



LabViewInterface::LabViewInterface(PCSTR IP, int PORT)
	:vf(vector<float>(10))
{
	std::cerr << "labview_interface: constructing" << std::endl;

	//winsock2の初期化
	WSAStartup(MAKEWORD(2, 0), &wsaData);
	//ソケットの作成
	sock = socket(AF_INET, SOCK_STREAM, 0);

	//接続先指定用構造体の準備
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(PORT);
	//server.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
	InetPton(AF_INET, IP, &server_addr.sin_addr.s_addr);


	//サーバに接続
	connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr));

	memset(buf, 0, MAX_BUF_SIZE);
	send(sock, "Eabc1;", MAX_BUF_SIZE, 0);
	recv(sock, buf, MAX_BUF_SIZE, 0);
	std::cerr << "first buf " << buf <<"#####"<< std::endl;

	constructed = true;
	std::cerr << "labview_interface: constructed" << std::endl;

}


std::vector<float> split(std::string s, int n) {
	if (s.size() < 2) return std::vector<float>(10);
	s = s.substr(2);
	vector<float> ret(n);
	int cnt = 0;
	std::string tmp;
	for (int i = 0; i < s.size(); i++) {
		if (s[i] == ',') {
			ret[cnt] = stof(tmp);
			tmp = "";
			cnt++;
		}
		else {
			tmp += s[i];
		}
		if (cnt > n) break;
	}
	if (tmp != "") ret[n - 1] = stof(tmp);
	return ret;
}


void LabViewInterface::update() {

//	std::cerr << "labview_interface: ";

	if (!constructed) return;

	//サーバからデータを受信
	send(sock, "Eabc1;", MAX_BUF_SIZE, 0);
	recv(sock, buf, MAX_BUF_SIZE, 0);

	vf = split(buf, 10);

	///////std::cerr <<"buf"<<buf << std::endl;
	//for (int i = 0; i < 10; i++) std::cerr << vf[i] << " ";
	//std::cerr << std::endl;

	for (int i = 0; i < 3; i++) vf[i] /= 1000.;

	for (int i = 3; i < 6; i++) vf[i] *= M_PI / 180.;

}


LabViewInterface::~LabViewInterface() {
	if (!constructed) return;
	closesocket(sock);
	WSACleanup();
}

