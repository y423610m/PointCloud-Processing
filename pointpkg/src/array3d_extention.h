/*
array<double, 3>の足し算，引き算，掛け算，スカラー倍などの演算拡張

array + array
array - array
array x array 外積
array * array 内積
array * scholor
array / scholor
arrayのノルム
arrayの単位ベクトル
array2d*array
RotX,Y,Z
出力
*/

#pragma once

////////////////////// array<double, 3>同士の演算
array<double, 3> operator+(const array<double, 3>& a, const array<double, 3>& b) {
	array<double, 3> ret;
	for (int i = 0; i < 3; i++) ret[i] = a[i] + b[i];
	return ret;
}

array<double, 3> operator-(const array<double, 3>& a, const array<double, 3>& b) {
	array<double, 3> ret;
	for (int i = 0; i < 3; i++) ret[i] = a[i] - b[i];
	return ret;
}

array<double, 3> outer_prod(const array<double, 3>& a, const array<double, 3>& b) {
	array<double, 3> ret;
	for (int i = 0; i < 3; i++) {
		ret[i] += a[(i + 1) % 3] * b[(i + 2) % 3];
		ret[i] -= a[(i + 2) % 3] * b[(i + 1) % 3];
	}
	return ret;
}

double inner_prod(const array<double, 3>& a, const array<double, 3>& b) {
	double ret;
	for (int i = 0; i < 3; i++) ret += a[i] * b[i];
	return ret;
}

////////////////////// array<double, 3>とスカラー
array<double, 3> operator*(const array<double, 3>& a, const double s) {
	array<double, 3> ret;
	for (int i = 0; i < 3; i++) ret[i] = a[i]*s;
	return ret;
}

array<double, 3> operator/(const array<double, 3>& a, const double s) {
	if (s == 0.0) return a;
	array<double, 3> ret;
	for (int i = 0; i < 3; i++) ret[i] = a[i] / s;
	return ret;
}

//ノルム
double norm(const array<double, 3>& a) {
	double ret = 0.;
	for (int i = 0; i < 3; i++) ret += a[i] * a[i];
	return sqrt(ret);
}

//単位ベクトル
array<double, 3> unit(const array<double, 3>& a) {
	double Norm = norm(a);
	if (Norm == 0.0) return a;
	array<double, 3> ret;
	for (int i = 0; i < 3; i++) ret[i] = a[i] / Norm;
	return ret;
}

//array2d * array 行列とベクトルの積
array<double, 3> operator*(const array<array<double, 3>,3> A, const array<double, 3>& b) {
	array<double, 3> ret;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			ret[i] += A[i][j] * b[j];
		}
	}
	return ret;
}

array<array<double, 3>, 3> RotX(double th) {
	array<array<double, 3>, 3> ret;
	double c = cos(th);
	double s = sin(th);
	ret[0] = { 1,0,0. };
	ret[1] = { 0,c,-s };
	ret[2] = { 0,s,c };
	return ret;
}

array<array<double, 3>, 3> RotY(double th) {
	array<array<double, 3>, 3> ret;
	double c = cos(th);
	double s = sin(th);
	ret[0] = { c,0,s };
	ret[1] = { 0,1,0. };
	ret[2] = { -s,0,c };
	return ret;
}

array<array<double, 3>, 3> RotZ(double th) {
	array<array<double, 3>, 3> ret;
	double c = cos(th);
	double s = sin(th);
	ret[0] = { c,-s,0. };
	ret[1] = { s,c,0. };
	ret[2] = { 0,0,1. };
	return ret;
}

//output
//vector output 
ostream& operator<<(ostream& os, const array<double, 3>& arr) {
	//for (auto x : arr) os << x << " ";
	for (int i = 0; i < 3;i++) os << arr[i] << " ";
	return os;
}