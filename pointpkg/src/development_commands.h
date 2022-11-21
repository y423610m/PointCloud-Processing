#pragma once

#include <iostream>
#include <vector>
using namespace std;

#define DEBUG_ON 1
    #ifdef DEBUG_ON
    #define PS(a) cerr<<a<<" ";
    #define PL(a) cerr<<a<<endl;
    #define NAME(a) #a
    #define ES(a) cerr<<NAME(a)<<": "<<a<<" ";
    #define EL(a) cerr<<NAME(a)<<": "<<a<<endl;
#elif
    #define PS(a) while(0){}
    #define PL(a) while(0){}
    #define ES(a) while(0){}
    #define EL(a) while(0){}
#endif
//–ˆ‰ñ‚Ìflush–hŽ~
//#define endl "\n"

//vector output 
template <typename T>
ostream& operator<<(ostream& os, const vector<T>& vec) {
    for (auto x : vec) os << x << " ";
    return os;
}

template<typename T, typename U>
void chmin(T& A, const U& B) {
    if (A > B) A = B;
}

template<typename T, typename U>
void chmax(T& A, const U& B) {
    if (A < B) A = B;
}