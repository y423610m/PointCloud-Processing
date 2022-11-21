/*

学習:rosrun pointpkg svm train InputModelName ./data.in OutputModelName 
推論:rosrun pointpkg svm test InputModelName ./data.in
if InputModelName == NEW create new Model

例
rosrun pointpkg svm_train train model_result.out ./data.in
rosrun pointpkg svm_train test model_result.out ./data.in

data.in:pcwsからの相対パスor絶対パス.は以下．
N:データサイズ
d:データ
L:ラベル(1or0)

N
d1_1 d1_2 d1_3 d1_4 L1
d2_1 d2_2 d2_3 d2_4 L2
.
.
.
dN_1 dN_2 dN_3 dN_4 LN
*/



#include "development_commands.h"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/ml.hpp>
using namespace cv;
using namespace cv::ml;

#include <vector>
#include <string>
#include <fstream>
using namespace std;

vector<string> readfile(string filename) {
	vector<string> ret;
	ifstream ifs(filename);
	string line;
	while (getline(ifs, line)) {
		ret.push_back(line);
	}
	return ret;
}

vector<string> split(const string& s_in, string split = ",") {
	vector<string> ret;
	string tmp;
	for (int i = 0; i < (int)s_in.size(); i++) {
		bool found = false;
		for (int j = 0; j < (int)split.size(); j++) {
			if (s_in[i] == split[j]) {
				if (tmp != "") { ret.push_back(tmp); }
				tmp = "";
				found = true;
				break;
			}
		}
		if (!found) tmp += s_in[i];
	}
	if (tmp != "") ret.push_back(tmp);
	return ret;
}

int main(int argc, char** argv) {

	EL(argv[1]);
	string task;
	string InputModel;
	string datafile;
	string OutputModel;
	if (argc > 1) task = argv[1];
	if (argc > 2) InputModel = argv[2];
	if (argc > 3) datafile = argv[3];
	if (argc > 4) OutputModel = argv[4];


	Ptr<SVM> svm;
	if (InputModel =="") {
		svm = SVM::create();
		svm->setType(SVM::C_SVC);
		svm->setKernel(SVM::LINEAR);
		svm->setTermCriteria(TermCriteria(TermCriteria::MAX_ITER, 100, 1e-6));
		EL("created New SVM Model")
	}
	else {
		svm = SVM::create();
		svm = SVM::load(InputModel);
		ES(InputModel) EL("Loaded");
	}



	if (datafile == "") {
		EL("No File");
		return 0;
	}

	vector<string> file = readfile(datafile);
	EL(file.size());
	int N = stoi(file[0]);
	EL(N);

	vector<int> labels;
	vector<float> data;
	labels.reserve(N);
	data.reserve(4 * N);
	vector<string> line;
	for (int i = 0; i < N; i++) {
		line = split(file[i + 1], " ");
		for (int j = 0; j < 4; j++) {
			data.emplace_back(stof(line[j]));
		}
		labels.emplace_back(stoi(line.back()));
	}
	Mat Labels(N, 1, CV_32SC1, &labels[0]);
	Mat Data(N, 4, CV_32F, &data[0]);

	if(false){//２次元描画 デバッグ用
		int W = 512, H = 640;
		Mat Image = Mat::zeros(H, W, CV_8UC3);
		Vec3b green(0, 255, 0), blue(255, 0, 0), red(0, 0, 255);
		for (int h = 0; h < H; h++) {
			for (int w = 0; w < W; w++) {
				//ES(h) ES(w)
				Mat Sample = (Mat_<float>(1, 4) << w, h, w, h);
				float res = svm->predict(Sample);
				//EL(res)
				if (res == 1) Image.at<Vec3b>(h, w) = green;
				else if (res == -1) Image.at<Vec3b>(h, w) = blue;
				else Image.at<Vec3b>(h, w) = red;
			}
		}
		EL("c");

		for (int i = 0; i < N; i++) {
			circle(Image, Point(data[i * 4 + 0], data[i * 4 + 1]), 5, Scalar(labels[i], labels[i], labels[i]) * 128 + Scalar(1, 1, 1) * 128, 1);
		}

		imshow("result", Image);
		waitKey(0);
	}

	if (task == "train") {
		svm->train(Data, ROW_SAMPLE, Labels);
	}

	//精度検証
	if (task == "test") {
		int cnt[2][2];
		for (int i = 0; i < 2; i++) for (int j = 0; j < 2; j++) cnt[i][j] = 0;
		for (int i = 0; i < N; i++) {
			Mat Sample = (Mat_<float>(1, 4) << data[i * 4], data[i * 4 + 1], data[i * 4 + 2], data[i * 4 + 3]);
			float res = svm->predict(Sample);
			cnt[labels[i]][res == 1]++;
		}
		/*　	予測
		*		偽　真
		実偽
		　真
		*/
		cerr << "正解　推論　数" << endl;
		for (int i = 0; i < 2; i++) for (int j = 0; j < 2; j++){
			cerr << i << "    " << j << "    " << cnt[i][j] << endl;
		}
		double Precision = 1.0 * (cnt[1][1]) / (cnt[0][1] + cnt[1][1]);
		double Recall = 1.0 * (cnt[1][1]) / (cnt[1][0] + cnt[1][1]);
		EL(Precision)
		EL(Recall)
	}


	if (task == "train") {
		if (OutputModel == "") OutputModel = "model_result.out";
		svm->save(OutputModel);
		ES(OutputModel) EL("Saved");
	}


	//svm->save("")

	return 0;
}