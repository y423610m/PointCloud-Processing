/*
array<double, 3>�̑����Z�C�����Z�C�|���Z�C�X�J���[�{�Ȃǂ̉��Z�g��

array + array
array - array
array x array �O��
array * array ����
array * scholor
array / scholor
array�̃m����
array�̒P�ʃx�N�g��
array2d*array
RotX,Y,Z
�o��
*/

#pragma once

////////////////////// array<double, 3>���m�̉��Z
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

////////////////////// array<double, 3>�ƃX�J���[
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

//�m����
double norm(const array<double, 3>& a) {
	double ret = 0.;
	for (int i = 0; i < 3; i++) ret += a[i] * a[i];
	return sqrt(ret);
}

//�P�ʃx�N�g��
array<double, 3> unit(const array<double, 3>& a) {
	double Norm = norm(a);
	if (Norm == 0.0) return a;
	array<double, 3> ret;
	for (int i = 0; i < 3; i++) ret[i] = a[i] / Norm;
	return ret;
}

//array2d * array �s��ƃx�N�g���̐�
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