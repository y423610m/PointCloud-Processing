/*
array<T, 3>‚Ì‘«‚µZCˆø‚«ZCŠ|‚¯ZCƒXƒJƒ‰[”{‚È‚Ç‚Ì‰‰ZŠg’£

array + array
array - array
array x array ŠOÏ
array * array “àÏ
array * scholor
array / scholor
array‚Ìƒmƒ‹ƒ€
array‚Ì’PˆÊƒxƒNƒgƒ‹
array2d*array
RotX,Y,Z
o—Í
*/

#pragma once

////////////////////// array<T, 3>“¯m‚Ì‰‰Z
template<typename T>
array<T, 3> operator+(const array<T, 3>& a, const array<T, 3>& b) {
	array<T, 3> ret;
	for (int i = 0; i < 3; i++) ret[i] = a[i] + b[i];
	return ret;
}

template<typename T>
array<T, 3> operator-(const array<T, 3>& a, const array<T, 3>& b) {
	array<T, 3> ret;
	for (int i = 0; i < 3; i++) ret[i] = a[i] - b[i];
	return ret;
}

template<typename T>
array<T, 3> outer_prod(const array<T, 3>& a, const array<T, 3>& b) {
	array<T, 3> ret;
	for (int i = 0; i < 3; i++) {
		ret[i] = 0.0;
		ret[i] += a[(i + 1) % 3] * b[(i + 2) % 3];
		ret[i] -= a[(i + 2) % 3] * b[(i + 1) % 3];
	}
	return ret;
}

template<typename T>
T inner_prod(const array<T, 3>& a, const array<T, 3>& b) {
	T ret = 0.0;
	for (int i = 0; i < 3; i++) ret += a[i] * b[i];
	return ret;
}

////////////////////// array<T, 3>‚ÆƒXƒJƒ‰[
template<typename T, typename U>
array<T, 3> operator*(const array<T, 3>& a, const U s) {
	array<T, 3> ret;
	for (int i = 0; i < 3; i++) ret[i] = a[i]*s;
	return ret;
}

template<typename T, typename U>
array<T, 3> operator/(const array<T, 3>& a, const U s) {
	if (s == 0.0) return a;
	array<T, 3> ret;
	for (int i = 0; i < 3; i++) ret[i] = a[i] / s;
	return ret;
}

//ƒmƒ‹ƒ€
template<typename T>
T norm(const array<T, 3>& a) {
	T ret = 0.;
	for (int i = 0; i < 3; i++) ret += a[i] * a[i];
	return sqrt(ret);
}

//’PˆÊƒxƒNƒgƒ‹
template<typename T>
array<T, 3> unit(const array<T, 3>& a) {
	T Norm = norm(a);
	if (Norm == 0.0) return a;
	array<T, 3> ret;
	for (int i = 0; i < 3; i++) ret[i] = a[i] / Norm;
	return ret;
}

//array2d * array s—ñ‚ÆƒxƒNƒgƒ‹‚ÌÏ
template<typename T>
array<T, 3> operator*(const array<array<T, 3>,3> A, const array<T, 3>& b) {
	array<T, 3> ret;
	for (int i = 0; i < 3; i++) {
		ret[i] = 0.0;
		for (int j = 0; j < 3; j++) {
			ret[i] += A[i][j] * b[j];
		}
	}
	return ret;
}

template<typename T>
array<array<T, 3>, 3> RotX(T th) {
	array<array<T, 3>, 3> ret;
	T c = cos(th);
	T s = sin(th);
	ret[0] = { 1,0,0. };
	ret[1] = { 0,c,-s };
	ret[2] = { 0,s,c };
	return ret;
}

template<typename T>
array<array<T, 3>, 3> RotY(T th) {
	array<array<T, 3>, 3> ret;
	T c = cos(th);
	T s = sin(th);
	ret[0] = { c,0,s };
	ret[1] = { 0,1,0. };
	ret[2] = { -s,0,c };
	return ret;
}

template<typename T>
array<array<T, 3>, 3> RotZ(T th) {
	array<array<T, 3>, 3> ret;
	T c = cos(th);
	T s = sin(th);
	ret[0] = { c,-s,0. };
	ret[1] = { s,c,0. };
	ret[2] = { 0,0,1. };
	return ret;
}

//output
//vector output 
template<typename T>
ostream& operator<<(ostream& os, const array<T, 3>& arr) {
	//for (auto x : arr) os << x << " ";
	for (int i = 0; i < 3;i++) os << arr[i] << " ";
	return os;
}